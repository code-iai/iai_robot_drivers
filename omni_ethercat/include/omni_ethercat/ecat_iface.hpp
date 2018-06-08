/*
 * This file is part of the omni_ethercat project.
 *
 * Copyright (C) 2012-2016 Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de>
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

#ifndef OMNI_ETHERCAT_ECAT_IFACE_HPP
#define OMNI_ETHERCAT_ECAT_IFACE_HPP

#include <pthread.h>
#include <memory>  //For the shared_ptr

#include <ecrt.h>  //part of igh's ethercat master
#include "omni_ethercat/omnilib.hpp"
#include "omni_ethercat/interpolator.hpp"


//This order must match *types below
#define EC_INT8   0
#define EC_UINT8  1
#define EC_INT16  2
#define EC_UINT16 3
#define EC_INT32  4
#define EC_UINT32 5


namespace omni_ecat {


    void print_pdo_entry_reg(ec_pdo_entry_reg_t reg);

//Adjust for different VID/PID
    const uint32_t elmo_vendor_id = 0x0000009a;
    const uint32_t elmo_gold_whistle_product_id = 0x00030924;

    class EcatPdoEntry {
    public:
        uint16_t index;
        uint8_t subindex;
        unsigned int *offset;
        unsigned int *bit_position;
    };

//Memory to hold the offsets to the individual variables of the Process Data
//(Endianness could differ from CPU depending on architecture, so use the EC_READ, EC_WRITE macros to get data in and out)
//They are all unsigned int here (and (pd_domain + offset_var) is a uint8_t*),
// but the data gets casted to the right types using the mentioned EC_READ/EC_WRITE macros
    class DriveProcessDataOffsets {
    public:
        //vars that we write to the Drives
        unsigned int w_target_velocity;
        unsigned int w_controlword;
        unsigned int w_mode_of_operation;
        unsigned int w_digital_outputs;
        //vars that we read from the Drives
        unsigned int r_actual_position;
        unsigned int r_digital_inputs;
        unsigned int r_actual_velocity;
        unsigned int r_statusword;
        unsigned int r_mode_of_operation_display;
        unsigned int r_actual_torque;
        unsigned int r_elmo_status_register;
        unsigned int r_dc_link_voltage;
    };

//Registers we read from the drives
//Has to match the r_* variables in the DriveProcessDataOffsets class
//Here the variables have their correct type
    class DriveTaskReadVars {
    public:
        int32_t actual_position;
        uint32_t digital_inputs;
        int32_t actual_velocity;
        uint16_t statusword;
        int8_t mode_of_operation_display;
        int16_t actual_torque;
        uint32_t elmo_status_register;
        uint32_t dc_link_voltage;
    };


//Registers that we will write on the drives
//Has to match the w_* variables in the DriveProcessDataOffsets class
//Here the variables have their correct type
    class DriveTaskWriteVars {
    public:
        //DriveTaskWriteVars();
        int32_t target_velocity;
        uint16_t controlword;
        int8_t mode_of_operation;
        uint32_t digital_outputs;
    };


//We will use one instance of EcatELMODrive per motor driver slave on the bus
    class EcatELMODrive {
    public:
        EcatELMODrive(std::string name, uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code);

        ~EcatELMODrive();

        void setup_variables();

        //TODO: Cleanup this interface, an move entries to be private

        //variables to identify the slave
        uint16_t alias_;
        uint16_t position_;
        uint32_t vendor_id_;
        uint32_t product_code_;
        std::string name_;

        ec_slave_config_t *sc;
        ec_slave_config_state_t slave_config_state;
        ec_slave_config_state_t slave_config_state_old; //rename to old_state


        std::vector<ec_pdo_entry_reg_t> get_pdo_entry_regs();

        void print_pdo_entry_regs();

        //Methods that copy in/out of the buffers
        void task_data_to_user_data();

        void user_data_to_task_data();

        //Methods that use EC_READ and EC_WRITE to get data out/in of the process variables
        void process_to_task_data();

        void task_data_to_process();

        void set_domain_pd(uint8_t *domain_pd);


        DriveTaskReadVars task_rdata_user_side;
        DriveTaskWriteVars task_wdata_user_side;
        bool old_sto_state;

    private:
        DriveTaskReadVars task_rdata_process_side;
        DriveTaskWriteVars task_wdata_process_side;

        DriveProcessDataOffsets pdata_offsets;   //memory the ECAT master writes to directly

        //add_pdo_entry stores a desired pdo mapping into pdos_
        void add_pdo_entry(uint16_t index, uint8_t subindex, unsigned int *offset, unsigned int *bit_position);

        //pdos_ holds a list of the partial pdo entry (to be completed with the variables that identify the slave)
        std::vector<EcatPdoEntry> pdos_;

        uint8_t *domain_pd_;  //holds the address to which the offsets are relative


    };

//Helper class to control the EtherCAT master, and talk to the slaves
    class EcatAdmin {
    public:
        EcatAdmin();

        int
        ecat_init();  //Should be the first thing to be called after instantiating. Check the return value to decide what to do afterwards.
        bool finished_ecat_init();  // Will return true if the ecat_init has finished happily

        void
        check_drive_state();   //should be called periodically to check on the state of the drives (and reactivates them if they went down)
        void shutdown();

        void print_pdo_entries();

        void ec_drives_vel_zero();

        std::map<std::string, std::shared_ptr<EcatELMODrive>> drive_map;  //holds the name and pointer to each drive

        void set_new_goal_twist(double dx, double dy, double dtheta);

        void interpolator_to_wheels();

        omni_ethercat::JacParams jac_params_;


    private:
        //Creates the pdo_entry table
        void prepare_objects_for_slaves_on_boxy();

        double max_wheel_speed_;


        int start_omni_realtime();

        int stop_omni_realtime();

        ReflexxesInterpolator interpolator; //Will interpolate in twist space

        void realtime_main();  //Function that will run in a thread with realtime priority
        void cyclic_ecat_task();  //gets called by realtime_main once per cycle, and does the ethercat regular chores
        int cyclic_counter;
        uint64_t ecat_cycle_counter;

        //FIXME: Need to add functionality for drives that need to home (like the torso one)
        //void configure_torso_drive();

        void ec_drives_poweroff();

        void ec_drives_speedcontrol();

        void ec_drives_recover();

        void ec_drives_poweron();


        unsigned int num_drives_;
        bool torso_present_;


        bool finished_ecat_init_;

        //helper function to write to registers using SDO
        int ecat_writeSDO(int device, int index, int subindex, int value, int type);

        void check_domain1_state();

        void check_master_state(void);

        void check_slave_config_states(void);


        std::vector<ec_pdo_entry_reg_t> get_pdo_entry_regs();

        std::vector<ec_pdo_entry_reg_t> get_pdo_entry_regs_terminated();


        //variables used in the realtimeMain
        int realtime_cycle_period_ns;    //This variable in nanoseconds defines the ethercat cycle time
        int rt_should_exit;
        int rt_misses;
        pthread_mutex_t rt_data_mutex;

        ec_master_t *ec_master = NULL;
        ec_master_state_t master_state;// = { };

        ec_domain_t *domain1 = NULL;
        ec_domain_state_t domain1_state;// = { };

        /* Process data */
        uint8_t *domain1_pd;    /* Process data memory */

        pthread_t rt_thread;

        //Entry function needed to create the realtime thread using pthread_create and point to a member
        static void *realtimeMainEntryFunc(void *This) {
            ((EcatAdmin *) This)->realtime_main();
            return NULL;
        }

        bool ec_domain_running;


    };


}


#endif // OMNI_ETHERCAT_ECAT_IFACE_HPP
