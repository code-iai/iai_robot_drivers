/*
 * This file is part of the omni_ethercat project.
 *
 * Copyright (C) 2012-2018 Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de>
 * Copyright (C) 2009-2012 Ingo Kresse <kresse@in.tum.de>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <omni_ethercat/ecat_iface.hpp>
#include <omni_ethercat/interpolator.hpp>
#include <ecrt.h>

Eigen::IOFormat CommaInitFmt2(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");


/*****************************************************************************/

/* PDO index, subindex, size in bits */
static ec_pdo_entry_info_t chosen_pdo_entries[] = {
        /* Write */
        //Using 0x1601, a good fit
        {0x60ff, 0x00, 32},                            //target velocity int32
        {0x6040, 0x00, 16},                            //control word uint16
        //using 0x1607, a configurable PDO
        {0x60fe, 0x01, 32},  //originally in 0x161D    //digital outputs uint32
        {0x6060, 0x00, 8},                             //mode of operation int8

        /* Read */
        //using 0x1a03, because it is a good fit, and then 0x1a07 a configurable PDO
        {0x6064, 0x00, 32},   //these go in 0x1a03  //position actual value int32
        {0x60fd, 0x00, 32},                         //digital inputs uint32
        {0x606c, 0x00, 32},                         //velocity actual value  int32
        {0x6041, 0x00, 16},                         //status word uint16
        //The following are in PDO 0x1a07
        {0x6061, 0x00, 8},    //originally in 0x1a0b   //modes of operation display int8
        {0x6077, 0x00, 16},   //originally in 0x1a13   //torque actual value int16  (unit is 1/1000 of rated torque)
        {0x1002, 0x00, 32},   //originally in 0x1a22   //ELMO status register
        {0x6079, 0x00, 32},  //originally in 0x1a18    //DC link circuit voltage in mV
};

/* PDO index, #entries, array of entries to map */
static ec_pdo_info_t chosen_pdos[] = {
        {0x1601, 2, chosen_pdo_entries + 0},
        {0x1607, 2, chosen_pdo_entries + 2},

        {0x1a03, 4, chosen_pdo_entries + 4},
        {0x1a07, 4, chosen_pdo_entries + 8},
};


/* Sync manager index, SM direction, #PDOs, arrays with PDOs to assign */
/* SM#, Input/Output, # of PDOs, address where they start, enable/disable  */
static ec_sync_info_t chosen_syncs[] = {
        //{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        //{1, EC_DIR_INPUT , 0, NULL, EC_WD_DISABLE},
        {2 /* SM2 */     , EC_DIR_OUTPUT, 2 /*# of PDOs*/, chosen_pdos + 0, EC_WD_ENABLE},
        {3 /* SM3 */     , EC_DIR_INPUT,  2,               chosen_pdos + 2, EC_WD_ENABLE},
        {0xff}
};

/*****************************************************************************/



static void timespecInc(struct timespec *tick, int nsec) {
    tick->tv_nsec += nsec;
    while (tick->tv_nsec >= 1e9) {
        tick->tv_nsec -= 1e9;
        tick->tv_sec++;
    }
}


namespace omni_ecat {


//Print in a human-readable form one of the ec_pdo_entry_reg_t
    void print_pdo_entry_reg(ec_pdo_entry_reg_t reg) {
        std::cout << "{" << reg.alias << ", " << reg.position << std::hex << " ,0x" << reg.vendor_id
                  << ", 0x" << reg.product_code << ", 0x" << reg.index << ", 0x" << static_cast<uint16_t>(reg.subindex)
                  << ", 0x"
                  << reg.offset << ", 0x" << reg.bit_position << "}" << std::dec << std::endl;

    }

    EcatAdmin::EcatAdmin() : rt_data_mutex(PTHREAD_MUTEX_INITIALIZER),
                             rt_should_exit(0), rt_misses(0), rt_thread(), ecat_cycle_counter(0),
                             ec_domain_running(false),
                             finished_ecat_init_(false), cyclic_counter(0),
                             realtime_cycle_period_ns(1e6), //make the realtime loop run at 1kHz
                             torso_present_(false) {

        std::cout << "EcatAdmin()" << std::endl;


        prepare_objects_for_slaves_on_boxy();
        //print_pdo_entries();


    }

    int EcatAdmin::start_omni_realtime() {

        std::cout << "start_omni_realtime()" << std::endl;
        auto pdo_entry_regs = get_pdo_entry_regs_terminated();


        if (!(ec_master = ecrt_request_master(0))) {
            std::cerr << "start_omni_realtime(): Could not request master 0" << std::endl;
            goto out_return;
        }

        std::cout << "start_omni_realtime(): Registering domain." << std::endl;
        if (!(domain1 = ecrt_master_create_domain(ec_master))) {
            std::cerr << "start_omni_realtime(): Domain creation failed." << std::endl;
            goto out_release_master;
        }


        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            auto &name = drive_el.first;
            /* master, (slave alias, slave position), (VID, PID) */
            if (!(drive->sc = ecrt_master_slave_config(ec_master, drive->alias_, drive->position_, elmo_vendor_id,
                                                       elmo_gold_whistle_product_id))) {
                std::cerr << "start_omni_realtime(): Could not get slave configuration for Drive at bus position = "
                          << drive->position_ << std::endl;
                goto out_release_master;
            }

            //Do not need to reconfigure the PDOs, because the current mapping is the standard on
            // You need to call this if you are changing the content of any of the PDOs (0x1a07,0x1a08 and 0x1607 and 0x1608)
            std::cout << "start_omni_realtime(): Configuring PDOs for motor at bus position = " << drive->position_
                      << std::endl;
            /* slave config, sync manager index, index of the PDO to assign */
            if (ecrt_slave_config_pdos(drive->sc, EC_END, chosen_syncs)) {
                std::cerr << "start_omni_realtime(): Could not configure PDOs for Drive at bus position = "
                          << drive->position_ << std::endl;
                goto out_release_master;
            }

        }

        std::cout << "start_omni_realtime(): Registering PDO entries..." << std::endl;

        //get_pdo_entry_regs_terminated() returns a  std::vector<ec_pdo_entry_reg_t>.
        // but since the std::vector has contiguous memory storage, getting the address
        // to the first element is the same as the address of the ec_pdo_entry_reg_t array[] that is needed
        // That array is also null terminated.


        if (ecrt_domain_reg_pdo_entry_list(domain1, &(pdo_entry_regs[0]))) {
            std::cerr << "start_omni_realtime(): Could not register PDO entries." << std::endl;
            goto out_release_master;
        }

        std::cout << "start_omni_realtime(): Activating the c_master." << std::endl;
        if (ecrt_master_activate(ec_master)) {
            std::cerr << "start_omni_realtime(): Could not activate the ethercat master" << std::endl;
            goto out_release_master;
        }
        /* Get internal process data for domain. */
        domain1_pd = ecrt_domain_data(domain1);

        // Tell this address to the objects holding the drive info, they'll need it for EC_READ_* and EC_WRITE_*
        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            drive->set_domain_pd(domain1_pd);
        }

        std::cout << "start_omni_realtime(): Starting the realtime thread" << std::endl;

        pthread_attr_t tattr;
        struct sched_param sparam;
        //The priority has to be high, but not higher than the priority of the IRQ thread of the ethernet device
        //Found it out using: NETDEVNAME=eth0; ps -A | awk "/irq\/[0-9]+-${NETDEVNAME}/ { print \$1 }" | xargs --no-run-if-empty
        //Then using chrt -p PIDNUMBER on the returned PID number. On my test system, it was prio=50, SCHED_FIFO.

        //sparam.sched_priority = sched_get_priority_max(SCHED_FIFO);
        sparam.sched_priority = 49;
        std::cout << "start_omni_realtime(): Maximum possible sched priority = " << sched_get_priority_max(SCHED_FIFO) << std::endl;
        std::cout << "start_omni_realtime(): Setting Sched priority for the realtime thread to = " << sparam.sched_priority << std::endl;

        pthread_attr_init(&tattr);
        pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
        pthread_attr_setschedparam(&tattr, &sparam);
        pthread_attr_setinheritsched(&tattr, PTHREAD_EXPLICIT_SCHED);

        if (pthread_create(&rt_thread, &tattr, realtimeMainEntryFunc, this) != 0) {
            std::cerr << "start_omni_realtime(): Error: Could not create realtime thread." << std::endl;
            goto out_release_master;
        }

        std::cout << "start_omni_realtime(): Thread started." << std::endl;
        return 0;

        out_release_master:
        std::cerr << "start_omni_realtime(): Releasing the master" << std::endl;
        ecrt_release_master(ec_master);
        out_return:
        std::cerr << "start_omni_realtime(): Failed to load. Aborting." << std::endl;
        return -1;

    }

    void EcatAdmin::shutdown() {

        //FIXME: Send signal to all velocities zero or something meaningful

        ec_drives_poweroff();   //turn the drives off

        //Stop the realtime ethercat thread
        stop_omni_realtime();


    }

    int EcatAdmin::stop_omni_realtime() {
        std::cout << "stop_omni_realtime()" << std::endl;

        //Signal a stop the realtime thread
        rt_should_exit = 1;
        //blocking wait for rt_thread to exit
        pthread_join(rt_thread, NULL);
        std::cout << "stop_omni_realtime(): rt_thread exited" << std::endl;

    }

    void EcatAdmin::set_new_goal_twist(double dx, double dy, double dtheta) {
        interpolator.set_target_twist(dx, dy, dtheta);
    }

    void EcatAdmin::interpolator_to_wheels() {
        // Do the Inverse Kinematics: Desired base twist -> 4 wheel velocities
        //This is a Vector holding 4 velocities, in this order of wheels: fl, fr, bl, br.
        //The wheel axes are chosen such that if the base is moving forward as a whole, all of them have positive rotations
        // imagine all wheels with the rotational axis pointing to the left (right-hand-rule)

        double dx, dy, dtheta;
        interpolator.get_next_twist(dx, dy, dtheta);

        omni_ethercat::Twist2d goal_twist = {dx, dy, dtheta};

        //std::cout << "new twist = [" << dx << "," << dy << "," << dtheta << "]" << std::endl;

        omni_ethercat::OmniEncVel vels;
        vels = omni_ethercat::omniIK(jac_params_, goal_twist);

        static omni_ethercat::OmniEncVel old_vels;
        //print if there was a change of velocities
        if (old_vels != vels) {
            old_vels = vels;
            //ROS_INFO_STREAM("Commanded wheel velocities: " << vels.format(CommaInitFmt2));
            std::cout << "Commanded wheel velocities: " << vels.format(CommaInitFmt2) << std::endl;
        }

        //Actually command the wheel velocities
        //omnilib uses this order: fl, fr, bl, br
        drive_map["fl"]->task_wdata_user_side.target_velocity = int32_t(vels[0]);
        drive_map["fr"]->task_wdata_user_side.target_velocity = int32_t(vels[1]);
        drive_map["bl"]->task_wdata_user_side.target_velocity = int32_t(vels[2]);
        drive_map["br"]->task_wdata_user_side.target_velocity = int32_t(vels[3]);


    }

    void EcatAdmin::realtime_main() {
        struct timespec tick;


        // Initialize the tick struct with the current time
        clock_gettime(CLOCK_REALTIME, &tick);

        //initialize the interpolator's pose
        interpolator.set_current_pose(0.0, 0.0, 0.0);


        while (!rt_should_exit) {


            interpolator_to_wheels();

            cyclic_ecat_task();  //do the things that need to happen regularly in the ECAT communication cycle

            //copy from/to buffers
            if (pthread_mutex_trylock(&rt_data_mutex) == 0) {
                for (auto &drive_el: drive_map) {
                    auto &drive = drive_el.second;
                    drive->user_data_to_task_data();  //setting variables to be written to ECAT
                    //drive->task_data_to_process();     //getting variables read from ECAT
                    drive->task_data_to_user_data(); //copy to the user amaldo FIXME
                }
                pthread_mutex_unlock(&rt_data_mutex);
            }




            // Compute end of next period
            timespecInc(&tick, realtime_cycle_period_ns);


            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);
            //FIXME: Check that the following comparison works (int vs float comparison)
            if ((now.tv_sec + static_cast<double>(now.tv_nsec) / 1e9) >
                (tick.tv_sec + static_cast<double>(tick.tv_nsec) / 1e9)) {
                // We overran, snap to next "period"
                timespecInc(&now, realtime_cycle_period_ns);
                tick = now;

                //record that we missed a deadline
                rt_misses++;
            }
            // Sleep until end of period
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

        }

        std::cout << "realtimeMain(): Finishing the rt thread." << std::endl;
        std::cout << "realtimeMain(): Had " << rt_misses << " misses." << std::endl;
        std::cout << "realtimeMain(): Had " << ecat_cycle_counter << " cycles." << std::endl;


    }


    bool EcatAdmin::get_global_sto_state() {

        bool global_sto_state = false;

        for (auto &drive_el: drive_map) {
            auto &name = drive_el.first;
            auto &drive = drive_el.second;

            global_sto_state = global_sto_state and drive->get_sto_from_status_reg();

        }

        return global_sto_state;

    }

    void EcatAdmin::check_drive_state() {
        //This should be called around once a second to bring the drives up if something happens, like E-Stop

        //After the EtherCAT communication is setup and PDOs are running (in OP mode), it is still necessary to enable the motor controller
        // which involves reseting errors, enabling voltage, switching on, etc
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 0
#define STATUSWORD_SWITCHED_ON_BIT 1
#define STATUSWORD_OPERATION_ENABLE_BIT 2
#define STATUSWORD_FAULT_BIT 3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 4
#define STATUSWORD_QUICK_STOP_BIT 5
#define STATUSWORD_SWITCH_ON_DISABLED_BIT 6
#define STATUSWORD_NO_USED_WARNING_BIT 7
#define STATUSWORD_ELMO_NOT_USED_BIT 8
#define STATUSWORD_REMOTE_BIT 9
#define STATUSWORD_TARGET_REACHED_BIT 10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT 11

        for (auto &drive_el: drive_map) {
            auto &name = drive_el.first;
            auto &drive = drive_el.second;

            unsigned int controlword = 0x00;
            unsigned int statusword = drive->task_rdata_user_side.statusword;
            //printf("check_drive_state(): m[%s]: mode of operation disp: %04x\n", name.c_str(), drive->task_rdata_user_side.mode_of_operation_display);
            //printf("check_drive_state(): m[%s]: ControlWord = %04x\n", name.c_str(), drive->task_wdata_user_side.controlword);

            bool sto_state = drive->get_sto_from_status_reg();


            if (sto_state == true) {
                //STO lets the drives work

                //printf("StatusWord[%s] = 0x%04x\n", name.c_str(), statusword);
                if (statusword & (1 << STATUSWORD_FAULT_BIT)) {
                    printf("\e[31;1mcheck_drive_state(): m[%s]: Has fault! <----------------\e[0m\n", name.c_str());
                }

                if (!(statusword & (1 << STATUSWORD_VOLTAGE_ENABLE_BIT))) {
                    printf("\e[31;1mcheck_drive_state(): m[%s]: Voltage not enabled! <----------------\e[0m\n",
                           name.c_str());
                }

                //if (!(statusword & (1 << STATUSWORD_SWITCH_ON_DISABLED_BIT))){
                //	   printf("\e[31;1mm%s Switch_on_disabled_bit <----------------\e[0m\n", name.c_str());
                //}


                if (!(statusword & (1 << STATUSWORD_OPERATION_ENABLE_BIT))) {
                    if (!(statusword & (1 << STATUSWORD_SWITCHED_ON_BIT))) {
                        if (!(statusword & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT))) {
                            if ((statusword & (1 << STATUSWORD_FAULT_BIT))) {
                                /* reset fault */
                                controlword = 0x80;
                                printf("\e[31;1mcheck_drive_state(): m[%s]: Reset fault <----------------\e[0m\n",
                                       name.c_str());
                            } else {
                                /* shutdown */
                                controlword = 0x06;
                                printf("\e[31;1mcheck_drive_state(): m[%s]: Shutdown <----------------\e[0m\n",
                                       name.c_str());
                            }
                        } else {
                            /* switch on */
                            controlword = 0x07;
                            printf("\e[31;1mcheck_drive_state(): m[%s]: Switch on <----------------\e[0m\n",
                                   name.c_str());
                        }
                    } else {
                        /* enable operation */
                        printf("\e[31;mcheck_drive_state(): m[%s] EnableOperation <----------------\e[0m\n",
                               name.c_str());
                        controlword = 0x0F;
                    }
                } else {
                    // All is good, continue
                    controlword = 0x0f;
                }
            } else {  //sto_state == false
                //Only print on sinking edge
                if (drive->old_sto_state == true) {
                    printf("\e[31;mcheck_drive_state(): m[%s]: STO is blocking!\e[0m\n", name.c_str());
                }
                controlword = 0x00; // don't do anything
            }

            drive->task_wdata_user_side.controlword = controlword;

            //std::cout << "check_drive_state(): m[" << name << "]: controlword = 0x" << std::hex << controlword << std::dec << std::endl;
            //save the STO state
            drive->old_sto_state = sto_state;

        }


    }

    int EcatAdmin::ecat_writeSDO(int slave_pos, int index, int subindex, int value, int type) {

        //std::cout << "ecat_writeSDO()" << std::endl;
        //abort if the master is not initialized
        if (ec_master == NULL) {
            std::cerr << "EcatAdmin::ecat_writeSDO(): Tried to use a NULL master. Aborting the call." << std::endl;
            return (-1);

        }

        //find the size of the variable to transmit
        int data_size = 0;
        switch (type) {
            case EC_INT8:
            case EC_UINT8:
                data_size = 1;
                break;
            case EC_INT16:
            case EC_UINT16:
                data_size = 2;
                break;
            case EC_INT32:
            case EC_UINT32:
                data_size = 4;
                break;
        }

        //Get the pointer as ecrt_master_sdo_download needs it
        uint8_t *data_addr = (uint8_t *) &value;

        int ret = -1;
        //Since this blocks forever if the cyclic_task is not running, we check first
        if (ec_domain_running)
            //send the SDO (blocking call)
            ret = ecrt_master_sdo_download(ec_master, slave_pos, index, subindex, data_addr, data_size, 0);
        else {
            std::cerr << "EcatAdmin::ecat_writeSDO(): Tried to write SDO without the domain running" << std::endl;
        }

        return (ret);

    }

    void EcatAdmin::ec_drives_speedcontrol() {
        std::cout << "ec_drives_speedcontrol()" << std::endl;


        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            // 0x03 for profiled velocity control (with internal trajectory generator)
            // 0x09 of cyclic synchronous velocity control (trajectory generation needs to happen on host)
            ecat_writeSDO(drive->position_, 0x6060, 0x00, 0x09, EC_INT8);
        }

    }


    void EcatAdmin::ec_drives_poweroff() {
        std::cout << "omnidrive_poweroff()" << std::endl;

        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            ecat_writeSDO(drive->position_, 0x6040, 0x00, 0x00, EC_UINT16);
        }

    }

    void EcatAdmin::ec_drives_recover() {
        std::cout << "omnidrive_recover()" << std::endl;

        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            ecat_writeSDO(drive->position_, 0x6040, 0x00, 0x80, EC_UINT16);
        }
    }


    void EcatAdmin::ec_drives_poweron() {
        std::cout << "omnidrive_poweron()" << std::endl;

        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            ecat_writeSDO(drive->position_, 0x6040, 0x00, 0x06, EC_UINT16);
        }

        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            ecat_writeSDO(drive->position_, 0x6040, 0x00, 0x07, EC_UINT16);
        }

        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            ecat_writeSDO(drive->position_, 0x6040, 0x00, 0x0f, EC_UINT16);
        }

    }

    int EcatAdmin::ecat_init() {
        std::cout << "ecat_init()" << std::endl;

        //TODO: Check version of the kernel module and the library match, using ecrt_version_magic()

        //FIXME: Does this avoid the reset after the initialization using SDOs?
        // Or we could avoid the SDO initialization altogether, and just do it after the PDO communication is up
        //for (auto & drive_el: drive_map) {
        //	auto & drive = drive_el.second;
        //	drive->task_wdata_user_side.controlword = 0x0f;
        //}



        if (start_omni_realtime() != 0) {
            std::cerr << "ecat_init(): Could not initialize the realtime thread" << std::endl;
            return -1;
        }


        //TODO: This can be moved to a member function
        bool init_took_too_long = true;
        //wait up to 3s for the ethercat BUS to come up
        for (unsigned int i = 0; i < 30; ++i) {
            if (ec_domain_running) {
                init_took_too_long = false;
                break;
            }
            usleep(1e5);  //0.1s = 1e5uS
        }

        if (init_took_too_long) {
            std::cerr << "ecat_init(): it took too long for the bus to come up (ec_domain_running). Aborting the init."
                      << std::endl;
            stop_omni_realtime();  //stop the rt thread before returning
            std::cerr << "ecat_init(): Releasing the master" << std::endl;
            ecrt_release_master(ec_master);
            return -1;
        }

        //Send some SDOs to get the drives in a good initial state
        ec_drives_poweroff();

        ec_drives_speedcontrol();
        //configure_torso_drive();
        ec_drives_recover();
        ec_drives_poweron();

        finished_ecat_init_ = true;
        std::cout << "ecat_init(): Returning happy" << std::endl;

        return 0;

    }

    bool EcatAdmin::finished_ecat_init() {
        return (finished_ecat_init_);
    }

    void EcatAdmin::check_domain1_state() {
        ec_domain_state_t ds;

        ecrt_domain_state(domain1, &ds);

        if (ds.working_counter != domain1_state.working_counter)
            printf("check_domain1_state(): Domain1: WorkingCounter = %u.\n", ds.working_counter);
        if (ds.wc_state != domain1_state.wc_state) {
            printf("check_domain1_state(): Domain1: State = %u.\n", ds.wc_state);

            if (ds.wc_state == EC_WC_COMPLETE) {
                std::cout << "check_domain1_state(): All registered process data were exchanged" << std::endl;
                ec_domain_running = true;
            } else {
                ec_domain_running = false;
            }
        }

        //save the state of the domain to the member variable
        domain1_state = ds;

    }

    void EcatAdmin::cyclic_ecat_task() {
        unsigned int i;
        //Increase the counter
        ecat_cycle_counter++;

        /* Receive process data. */
        ecrt_master_receive(ec_master);
        ecrt_domain_process(domain1);

        /* Check process data state (optional). */
        check_domain1_state();


        //Actually get data from the EtherCAT frames
        //Info about the data type and address found in MAN-CAN402IG.pdf from Elmo


        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            drive->process_to_task_data();  //This calls EC_READ on all the process variables
        }

        if (cyclic_counter > 0) {
            cyclic_counter--;
        } else if (finished_ecat_init_) {        /* Do this at 1 Hz */
            cyclic_counter = 1000;

            /* Check for master state (optional). */
            check_master_state();

            /* Check for slave configuration state(s) (optional). */
            check_slave_config_states();
        }

        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            drive->task_data_to_process(); //This calls EC_WRITE on all the variables
            //std::cout << "[" << i << "] " << "controlword = 0x" << std::hex << drive->task_wdata_user_side.controlword << std::dec << std::endl;
        }

        /* Send process data. */
        ecrt_domain_queue(domain1);
        ecrt_master_send(ec_master);

    }


    void EcatAdmin::check_master_state(void) {
        ec_master_state_t ms;

        ecrt_master_state(ec_master, &ms);

        //Report if something has changed from the saved state
        if (ms.slaves_responding != master_state.slaves_responding)
            printf("check_master_state(): %u slave(s).\n", ms.slaves_responding);
        if (ms.al_states != master_state.al_states)
            printf("check_master_state(): AL states: 0x%02X.\n", ms.al_states);
        if (ms.link_up != master_state.link_up)
            printf("check_master_state(): Link is %s.\n",
                   ms.link_up ? "up" : "down");

        //member_variable holding the state of the master after each check
        master_state = ms;

    }

/*****************************************************************************/

    void EcatAdmin::check_slave_config_states(void) {
        ec_slave_config_state_t s;

        //Report if something has changed from the saved slave_config_state
        for (auto &drive_el: drive_map) {
            auto &drive = drive_el.second;
            ecrt_slave_config_state(drive->sc, &s);
            if (s.al_state != drive->slave_config_state_old.al_state)
                printf("check_slave_config_states(): m[%s]: State 0x%02X.\n", drive->name_.c_str(), s.al_state);
            if (s.online != drive->slave_config_state_old.online)
                printf("check_slave_config_states(): m[%s]: %s.\n", drive->name_.c_str(),
                       s.online ? "online" : "offline");
            if (s.operational != drive->slave_config_state_old.operational)
                printf("check_slave_config_states(): m[%s]: %soperational.\n", drive->name_.c_str(),
                       s.operational ? "" : "Not ");

            //Save data to the drive object
            //TODO: Use this to check on the state of the slaves (that they are all OP)
            drive->slave_config_state = s;
            drive->slave_config_state_old = s;

        }

    }


    void EcatAdmin::prepare_objects_for_slaves_on_boxy() {


        //Here we instantiate one object EcatELMODrive per drive
        //These have memory addresses that get passed to the ethercat master, so they must stay valid
        //So we create them into shared_pointers
        //EtherCAT chain: 0:br:elmo-duo-back-A1   1:bl:elmo-duo-back-A2 2:fr:elmo-duo-front-A1  3:fl:elmo-duo-front-A2

        std::shared_ptr<EcatELMODrive> slave_fr = std::make_shared<EcatELMODrive>(std::string("fr"), 0, 2,
                                                                                  elmo_vendor_id,
                                                                                  elmo_gold_whistle_product_id);
        std::shared_ptr<EcatELMODrive> slave_fl = std::make_shared<EcatELMODrive>(std::string("fl"), 0, 3,
                                                                                  elmo_vendor_id,
                                                                                  elmo_gold_whistle_product_id);
        std::shared_ptr<EcatELMODrive> slave_br = std::make_shared<EcatELMODrive>(std::string("br"), 0, 0,
                                                                                  elmo_vendor_id,
                                                                                  elmo_gold_whistle_product_id);
        std::shared_ptr<EcatELMODrive> slave_bl = std::make_shared<EcatELMODrive>(std::string("bl"), 0, 1,
                                                                                  elmo_vendor_id,
                                                                                  elmo_gold_whistle_product_id);

        if (torso_present_) {
            std::shared_ptr<EcatELMODrive> slave_torso = std::make_shared<EcatELMODrive>(std::string("torso"), 0, 4,
                                                                                         elmo_vendor_id,
                                                                                         elmo_gold_whistle_product_id);
            drive_map[slave_torso->name_] = slave_torso;
        }

        //drive_map holds all the slaves for easy global configuration
        drive_map[slave_fl->name_] = slave_fl;
        drive_map[slave_fr->name_] = slave_fr;
        drive_map[slave_bl->name_] = slave_bl;
        drive_map[slave_br->name_] = slave_br;


    }

//The ethercat Master wants to have the array of ec_pdo_entry_reg_t terminated by a null entry, we add it here to the bottom
    std::vector<ec_pdo_entry_reg_t> EcatAdmin::get_pdo_entry_regs_terminated() {
        std::vector<ec_pdo_entry_reg_t> regs;
        regs = get_pdo_entry_regs();
        //Important: The = {} makes sure it is initialized empty, without it, it is random RAM
        ec_pdo_entry_reg_t terminator = {};
        memset(&terminator, 0, sizeof(terminator));
        regs.push_back(terminator);
        return (regs);
    }

//Returns a vector of all the ec_pdo_entry_reg_t for all the drives.
    std::vector<ec_pdo_entry_reg_t> EcatAdmin::get_pdo_entry_regs() {
        std::vector<ec_pdo_entry_reg_t> regs;

        for (auto const &drive: drive_map) {
            std::vector<ec_pdo_entry_reg_t> slave_entries;
            slave_entries = drive.second->get_pdo_entry_regs();


            for (auto entry: slave_entries) {
                regs.push_back(entry);
            }

        }

        return regs;

    }

    void EcatAdmin::print_pdo_entries() {
        std::vector<ec_pdo_entry_reg_t> entries = get_pdo_entry_regs();

        for (auto entry: entries) {
            print_pdo_entry_reg(entry);
        }
    }

    void EcatAdmin::ec_drives_vel_zero() {
        std::cout << "ec_drives_vel_zero()" << std::endl;

        for (auto &drive: drive_map) {
            drive.second->task_wdata_user_side.target_velocity = 0;
        }
    }


    EcatELMODrive::EcatELMODrive(std::string name, uint16_t alias, uint16_t position, uint32_t vendor_id,
                                 uint32_t product_code) : name_(name), alias_(alias), position_(position),
                                                          vendor_id_(vendor_id), product_code_(product_code) {
        //std::cout << "EcatSlave constructor" << std::endl;

        setup_variables();

        //zero out all the variables, specially useful to have initial speed equal zero
        task_rdata_user_side = {};
        task_rdata_process_side = {};
        task_wdata_process_side = {};
        task_wdata_user_side = {};

        old_sto_state = false;
    }

    EcatELMODrive::~EcatELMODrive() {
        std::cout << "~EcatELMODrive. index=" << position_ << std::endl;
    }

    void
    EcatELMODrive::add_pdo_entry(uint16_t index, uint8_t subindex, unsigned int *offset, unsigned int *bit_position) {
        EcatPdoEntry entry;
        entry.index = index;
        entry.subindex = subindex;
        entry.offset = offset;
        entry.bit_position = bit_position;

        pdos_.push_back(entry);
    }

    std::vector<ec_pdo_entry_reg_t> EcatELMODrive::get_pdo_entry_regs() {
        std::vector<ec_pdo_entry_reg_t> vec;

        for (auto &p: pdos_) {
            ec_pdo_entry_reg_t entry;

            //Parts that are the same always for this slave
            entry.alias = alias_;
            entry.position = position_;
            entry.vendor_id = vendor_id_;
            entry.product_code = product_code_;

            //Parts that change depending on the register
            entry.index = p.index;
            entry.subindex = p.subindex;
            entry.offset = p.offset;
            entry.bit_position = p.bit_position;

            vec.push_back(entry);

        }

        return (vec);


    }


    void EcatELMODrive::print_pdo_entry_regs() {

        std::vector<ec_pdo_entry_reg_t> data = get_pdo_entry_regs();

        for (auto &d: data) {
            print_pdo_entry_reg(d);
        }

    }

    void EcatELMODrive::setup_variables() {

        //Check that these match what 'ethercat cstruct' ouputs and the chosen_pdo_entries table
        add_pdo_entry(0x60FF, 0x0, &pdata_offsets.w_target_velocity, 0);
        add_pdo_entry(0x6040, 0x0, &pdata_offsets.w_controlword, 0);
        add_pdo_entry(0x6060, 0x0, &pdata_offsets.w_mode_of_operation, 0);
        add_pdo_entry(0x60FE, 0x01, &pdata_offsets.w_digital_outputs, 0);
        //
        add_pdo_entry(0x6064, 0x0, &pdata_offsets.r_actual_position, 0);
        add_pdo_entry(0x60FD, 0x0, &pdata_offsets.r_digital_inputs, 0);
        add_pdo_entry(0x606C, 0x0, &pdata_offsets.r_actual_velocity, 0);
        add_pdo_entry(0x6041, 0x0, &pdata_offsets.r_statusword, 0);
        add_pdo_entry(0x6061, 0x0, &pdata_offsets.r_mode_of_operation_display, 0);
        add_pdo_entry(0x6077, 0x0, &pdata_offsets.r_actual_torque, 0);
        add_pdo_entry(0x1002, 0x0, &pdata_offsets.r_elmo_status_register, 0);
        add_pdo_entry(0x6079, 0x0, &pdata_offsets.r_dc_link_voltage, 0);


    }

//Take data from process the process variables to the task variables_process_side
// This fixes the endianness if necessary, and casts to signed/unsigned variables of the correct bit-length
    void EcatELMODrive::process_to_task_data() {

        task_rdata_process_side.actual_position = EC_READ_S32(domain_pd_ + pdata_offsets.r_actual_position);
        task_rdata_process_side.digital_inputs = EC_READ_U32(domain_pd_ + pdata_offsets.r_digital_inputs);
        task_rdata_process_side.actual_velocity = EC_READ_S32(domain_pd_ + pdata_offsets.r_actual_velocity);
        task_rdata_process_side.statusword = EC_READ_U16(domain_pd_ + pdata_offsets.r_statusword);
        task_rdata_process_side.mode_of_operation_display = EC_READ_S8(
                domain_pd_ + pdata_offsets.r_mode_of_operation_display);
        task_rdata_process_side.actual_torque = EC_READ_S16(domain_pd_ + pdata_offsets.r_actual_torque);
        task_rdata_process_side.elmo_status_register = EC_READ_U32(domain_pd_ + pdata_offsets.r_elmo_status_register);
        task_rdata_process_side.dc_link_voltage = EC_READ_U32(domain_pd_ + pdata_offsets.r_dc_link_voltage);
    }

//Here we write to the process variables
    void EcatELMODrive::task_data_to_process() {
        EC_WRITE_S32(domain_pd_ + pdata_offsets.w_target_velocity, task_wdata_process_side.target_velocity);
        EC_WRITE_U16(domain_pd_ + pdata_offsets.w_controlword, task_wdata_process_side.controlword);
        EC_WRITE_S8(domain_pd_ + pdata_offsets.w_mode_of_operation, task_wdata_process_side.mode_of_operation);
        EC_WRITE_U32(domain_pd_ + pdata_offsets.w_digital_outputs, task_wdata_process_side.digital_outputs);
    }

//Take the data read from the process and put it into a struct for user's use
    void EcatELMODrive::task_data_to_user_data() {
        task_rdata_user_side = task_rdata_process_side;
    }

//Take the data the user has been writing to, and make it ready to write to ECAT in the next cycle
    void EcatELMODrive::user_data_to_task_data() {
        task_wdata_process_side = task_wdata_user_side;
    }

    void EcatELMODrive::set_domain_pd(uint8_t *domain_pd) {
        domain_pd_ = domain_pd;
    }

    bool EcatELMODrive::get_sto_from_status_reg() {
#define ELMO_STATUS_REG_STO1_BIT 14
#define ELMO_STATUS_REG_STO2_BIT 15
        //tested on CLI so: ethercat upload 0x1002 0 -p 0 -t uint32
        bool sto_state = (
                (task_rdata_user_side.elmo_status_register bitand (1 << ELMO_STATUS_REG_STO1_BIT)) and
                (task_rdata_user_side.elmo_status_register bitand (1 << ELMO_STATUS_REG_STO2_BIT)));

        //printf("check_drive_state(): m[%s]: sto_state = %i\n", name.c_str(), sto_state);
        //printf("check_drive_state(): m[%s]: ELMO_STATUS_REG = 0x%08x\n", name.c_str(), drive->task_rdata_user_side.elmo_status_register);
        return sto_state;

    }

//	  DriveTaskWriteVars::DriveTaskWriteVars(){
//		  std::cout << "DriveTaskWriteVars constructor. --------------------------" << std::endl;
//	  }



}
