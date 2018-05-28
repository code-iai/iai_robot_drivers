/*
 * This file is part of the omnimod project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *                    Ingo Kresse <kresse@in.tum.de>
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

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <pthread.h>

/****************************************************************************/

#include <ecrt.h>  //part of igh's ethercat master

/****************************************************************************/

#include "realtime.h"  // defines omniread_t, omniwrite_t

/*****************************************************************************/


	
#define FREQUENCY 1000
//NUM_DRIVES is defined in realtime.h

/* Optional features */
/*#define CONFIGURE_PDOS  1
#define EXTERNAL_MEMORY 1
#define SDO_ACCESS      0
*/

/*****************************************************************************/

static ec_master_t *master = NULL;
static ec_master_state_t master_state;// = { };

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state;// = { };

static ec_slave_config_t *sc[NUM_DRIVES];// = { NULL, NULL, NULL, NULL}; 
static ec_slave_config_state_t sc_state[NUM_DRIVES];// = {{}, {}, {}, {}};

static char prevent_set_position = 0;

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int exiting = 0;
static pthread_t thread;
static int misses=0;

/*****************************************************************************/

/* Process data */
static uint8_t *domain1_pd;	/* Process data memory */

/* Slave alias, slave position */
#define M0SlavePos 0, 0
#define M1SlavePos 0, 1
#define M2SlavePos 0, 2
#define M3SlavePos 0, 3
#define M4SlavePos 0, 4

/* Slave vendor ID, slave product code */
#define ELMOG  0x0000009a, 0x00030924


/*****************************************************************************/
/* PDO index, subindex, size in bits */
static ec_pdo_entry_info_t foo_pdo_entries[] = {
    /* Write */
    {0x6084, 0x00, 32}, //these go in 0x1607  //profile decelaration uint32
    {0x6083, 0x00, 32},                       //profile acceleration uint32
    {0x6081, 0x00, 32},                       //profile velocity uint32
    {0x60FE, 0x01, 32},                       //digital outputs uint32
    {0x607a, 0x00, 32}, //these go in 0x1605  //target position int32
    {0x60ff, 0x00, 32},                       //target velocity int32
    {0x6071, 0x00, 16},                       //target torque int16
    {0x6072, 0x00, 16},                       //max torque int16
    {0x6040, 0x00, 16},                       //control word uint16
    {0x6060, 0x00, 8},                        //mode of operation int8

    /* Read */
    {0x6064, 0x00, 32},   //these go in 0x1a07  //position actual value int32
    {0x60FD, 0x00, 32},                       //digital inputs uint32
    {0x606C, 0x00, 32},                       //velocity actual value  int32
    {0x6041, 0x00, 16},                       //status word uint16
    {0x6061, 0x00, 8},                        //modes of operation display int8
    {0x6077, 0x00, 16},                       //torque actual value int16  (unit is 1/1000 of rated torque)

};


/* PDO index, #entries, array of entries to map */
static ec_pdo_info_t foo_pdos[] = {
    {0x1607, 4, foo_pdo_entries + 0},
    {0x1605, 6, foo_pdo_entries + 4},
    {0x1a07, 6, foo_pdo_entries + 10},
};


/* Sync manager index, SM direction, #PDOs, arrays with PDOs to assign */
/* SM#, Input/Output, # of PDOs, address where they start, enable/disable  */
static ec_sync_info_t foo_syncs[] = {
    //{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    //{1, EC_DIR_INPUT , 0, NULL, EC_WD_DISABLE},
    {2 /* SM2 */     , EC_DIR_OUTPUT,   2 /*# of PDOs*/, foo_pdos + 0, EC_WD_ENABLE},
    {3 /* SM3 */     , EC_DIR_INPUT ,   1              , foo_pdos + 2, EC_WD_ENABLE},
    {0xff}
};
/*****************************************************************************/



/* Offsets for 'write' PDO entries */
//These have to be unsigned int
static unsigned int off_target_position[NUM_DRIVES];
static unsigned int off_target_velocity[NUM_DRIVES];
static unsigned int off_target_torque[NUM_DRIVES];
static unsigned int off_max_torque[NUM_DRIVES];
static unsigned int off_controlword[NUM_DRIVES];
static unsigned int off_mode_of_operation[NUM_DRIVES];
static unsigned int off_profile_velocity[NUM_DRIVES];
static unsigned int off_profile_acceleration[NUM_DRIVES];
static unsigned int off_profile_deceleration[NUM_DRIVES];
static unsigned int off_digital_outputs[NUM_DRIVES];

/* Offsets for 'read' PDO entries */
static unsigned int off_actual_position[NUM_DRIVES];
static unsigned int off_digital_inputs[NUM_DRIVES];
static unsigned int off_actual_velocity[NUM_DRIVES];
static unsigned int off_statusword[NUM_DRIVES];
static unsigned int off_mode_of_operation_display[NUM_DRIVES];
static unsigned int off_actual_torque[NUM_DRIVES];


/* (alias, position), (VID, PID), PDO entry index, PDO entry subindex, pointer, bit position */
static const ec_pdo_entry_reg_t domain1_regs[] = {
    //PDOs for writing to the controllers
    {M0SlavePos,  ELMOG, 0x607A, 0x00, &off_target_position[0]},
    {M1SlavePos,  ELMOG, 0x607A, 0x00, &off_target_position[1]},
    {M2SlavePos,  ELMOG, 0x607A, 0x00, &off_target_position[2]},
    {M3SlavePos,  ELMOG, 0x607A, 0x00, &off_target_position[3]},
    {M0SlavePos,  ELMOG, 0x60FF, 0x00, &off_target_velocity[0]},
    {M1SlavePos,  ELMOG, 0x60FF, 0x00, &off_target_velocity[1]},
    {M2SlavePos,  ELMOG, 0x60FF, 0x00, &off_target_velocity[2]},
    {M3SlavePos,  ELMOG, 0x60FF, 0x00, &off_target_velocity[3]},
    {M0SlavePos,  ELMOG, 0x6071, 0x00, &off_target_torque[0]},
    {M1SlavePos,  ELMOG, 0x6071, 0x00, &off_target_torque[1]},
    {M2SlavePos,  ELMOG, 0x6071, 0x00, &off_target_torque[2]},
    {M3SlavePos,  ELMOG, 0x6071, 0x00, &off_target_torque[3]},
    {M0SlavePos,  ELMOG, 0x6072, 0x00, &off_max_torque[0]},
    {M1SlavePos,  ELMOG, 0x6072, 0x00, &off_max_torque[1]},
    {M2SlavePos,  ELMOG, 0x6072, 0x00, &off_max_torque[2]},
    {M3SlavePos,  ELMOG, 0x6072, 0x00, &off_max_torque[3]},
    {M0SlavePos,  ELMOG, 0x6040, 0x00, &off_controlword[0]},
    {M1SlavePos,  ELMOG, 0x6040, 0x00, &off_controlword[1]},
    {M2SlavePos,  ELMOG, 0x6040, 0x00, &off_controlword[2]},
    {M3SlavePos,  ELMOG, 0x6040, 0x00, &off_controlword[3]},
    {M0SlavePos,  ELMOG, 0x6060, 0x00, &off_mode_of_operation[0]},
    {M1SlavePos,  ELMOG, 0x6060, 0x00, &off_mode_of_operation[1]},
    {M2SlavePos,  ELMOG, 0x6060, 0x00, &off_mode_of_operation[2]},
    {M3SlavePos,  ELMOG, 0x6060, 0x00, &off_mode_of_operation[3]},
    {M0SlavePos,  ELMOG, 0x6081, 0x00, &off_profile_velocity[0]},
    {M1SlavePos,  ELMOG, 0x6081, 0x00, &off_profile_velocity[1]},
    {M2SlavePos,  ELMOG, 0x6081, 0x00, &off_profile_velocity[2]},
    {M3SlavePos,  ELMOG, 0x6081, 0x00, &off_profile_velocity[3]},
    {M0SlavePos,  ELMOG, 0x6083, 0x00, &off_profile_acceleration[0]},
    {M1SlavePos,  ELMOG, 0x6083, 0x00, &off_profile_acceleration[1]},
    {M2SlavePos,  ELMOG, 0x6083, 0x00, &off_profile_acceleration[2]},
    {M3SlavePos,  ELMOG, 0x6083, 0x00, &off_profile_acceleration[3]},
    {M0SlavePos,  ELMOG, 0x6084, 0x00, &off_profile_deceleration[0]},
    {M1SlavePos,  ELMOG, 0x6084, 0x00, &off_profile_deceleration[1]},
    {M2SlavePos,  ELMOG, 0x6084, 0x00, &off_profile_deceleration[2]},
    {M3SlavePos,  ELMOG, 0x6084, 0x00, &off_profile_deceleration[3]},
    {M0SlavePos,  ELMOG, 0x6084, 0x00, &off_digital_outputs[0]},
    {M1SlavePos,  ELMOG, 0x6084, 0x00, &off_digital_outputs[1]},
    {M2SlavePos,  ELMOG, 0x6084, 0x00, &off_digital_outputs[2]},
    {M3SlavePos,  ELMOG, 0x6084, 0x00, &off_digital_outputs[3]},

    //From here are the PDOs for reading
    {M0SlavePos,  ELMOG, 0x6064, 0x00, &off_actual_position[0]},
    {M1SlavePos,  ELMOG, 0x6064, 0x00, &off_actual_position[1]},
    {M2SlavePos,  ELMOG, 0x6064, 0x00, &off_actual_position[2]},
    {M3SlavePos,  ELMOG, 0x6064, 0x00, &off_actual_position[3]},
    {M0SlavePos,  ELMOG, 0x60FD, 0x00, &off_digital_inputs[0]},
    {M1SlavePos,  ELMOG, 0x60FD, 0x00, &off_digital_inputs[1]},
    {M2SlavePos,  ELMOG, 0x60FD, 0x00, &off_digital_inputs[2]},
    {M3SlavePos,  ELMOG, 0x60FD, 0x00, &off_digital_inputs[3]},
    {M0SlavePos,  ELMOG, 0x606C, 0x00, &off_actual_velocity[0]},
    {M1SlavePos,  ELMOG, 0x606C, 0x00, &off_actual_velocity[1]},
    {M2SlavePos,  ELMOG, 0x606C, 0x00, &off_actual_velocity[2]},
    {M3SlavePos,  ELMOG, 0x606C, 0x00, &off_actual_velocity[3]},
    {M0SlavePos,  ELMOG, 0x6041, 0x00, &off_statusword[0]},
    {M1SlavePos,  ELMOG, 0x6041, 0x00, &off_statusword[1]},
    {M2SlavePos,  ELMOG, 0x6041, 0x00, &off_statusword[2]},
    {M3SlavePos,  ELMOG, 0x6041, 0x00, &off_statusword[3]},
    {M0SlavePos,  ELMOG, 0x6061, 0x00, &off_mode_of_operation_display[0]},
    {M1SlavePos,  ELMOG, 0x6061, 0x00, &off_mode_of_operation_display[1]},
    {M2SlavePos,  ELMOG, 0x6061, 0x00, &off_mode_of_operation_display[2]},
    {M3SlavePos,  ELMOG, 0x6061, 0x00, &off_mode_of_operation_display[3]},
    {M0SlavePos,  ELMOG, 0x6077, 0x00, &off_actual_torque[0]},
    {M1SlavePos,  ELMOG, 0x6077, 0x00, &off_actual_torque[1]},
    {M2SlavePos,  ELMOG, 0x6077, 0x00, &off_actual_torque[2]},
    {M3SlavePos,  ELMOG, 0x6077, 0x00, &off_actual_torque[3]},
    {}
};


static unsigned int counter = 0;

static int max_v = 100;

static omniwrite_t tar, tar_buffer;  /* Target velocities */
static omniread_t cur, cur_buffer;    /* Current velocities/torques/positions */


/*****************************************************************************/


void check_domain1_state(void)
{
	ec_domain_state_t ds;

	ecrt_domain_state(domain1, &ds);

	if (ds.working_counter != domain1_state.working_counter)
		printf("Domain1: WC %u.\n", ds.working_counter);
	if (ds.wc_state != domain1_state.wc_state)
		printf("Domain1: State %u.\n", ds.wc_state);

	domain1_state = ds;
	cur.working_counter = ds.working_counter;
	cur.working_counter_state = ds.wc_state;
}


/*****************************************************************************/


void check_master_state(void)
{
	ec_master_state_t ms;

	ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding)
		printf("%u slave(s).\n", ms.slaves_responding);
	if (ms.al_states != master_state.al_states)
		printf("AL states: 0x%02X.\n", ms.al_states);
	if (ms.link_up != master_state.link_up)
		printf("Link is %s.\n",
		       ms.link_up ? "up" : "down");

	master_state = ms;
	cur.master_link = ms.link_up;
	cur.master_al_states = ms.al_states;
	cur.master_slaves_responding = ms.slaves_responding;

}

/*****************************************************************************/

void check_slave_config_states(void)
{
	int i;

	ec_slave_config_state_t s;

    for (i = 0; i < NUM_DRIVES; i++) {
		ecrt_slave_config_state(sc[i], &s);
		if (s.al_state != sc_state[i].al_state)
			printf("m%d: State 0x%02X.\n", i, s.al_state);
		if (s.online != sc_state[i].online)
			printf("m%d: %s.\n", i,
			       s.online ? "online" : "offline");
		if (s.operational != sc_state[i].operational)
			printf("m%d: %soperational.\n", i,
			       s.operational ? "" : "Not ");
		sc_state[i] = s;

		cur.slave_state[i] = s.al_state;
		cur.slave_online[i] = s.online;
		cur.slave_operational[i] = s.operational;	
	}
}


/*****************************************************************************/

void cyclic_task()
{
	int i;

	/* Receive process data. */
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);

	/* Check process data state (optional). */
	check_domain1_state();


    //Actually get data from the EtherCAT frames
    //Info about the data type and address found in MAN-CAN402IG.pdf from Elmo
    for (i = 0; i < NUM_DRIVES; i++) {
        cur.position[i]          = EC_READ_S32(domain1_pd + off_actual_position[i]);
        cur.digital_inputs[i]    = EC_READ_U32(domain1_pd + off_digital_inputs[i]);
        cur.actual_velocity[i]   = EC_READ_S32(domain1_pd + off_actual_velocity[i]);
        cur.status[i]            = EC_READ_U16(domain1_pd + off_statusword[i]);
        cur.mode_of_operation_display[i] = EC_READ_S8(domain1_pd + off_mode_of_operation_display[i]);
        cur.actual_torque[i]     = EC_READ_S16(domain1_pd + off_actual_torque[i]);
	}
	

    // TODO: factor out these calls
	if (counter) {
		counter--;
	} else {		/* Do this at 1 Hz */
		counter = FREQUENCY;

        //printf("profile_vel = %d\n", tar.profile_velocity[4]);
        //printf("actual_velocity: %d", cur.actual_velocity[0]);
        //printf("actual_velocity: %d", cur.actual_velocity[4]);
        //int prof_vel = EC_READ_U32(domain1_pd + off_profile_velocity[4]);
        //printf("read profile_vel = %d\n",prof_vel);
        //int mode_of_op = EC_READ_S8(domain1_pd + off_mode_of_operation_display[4]);
        //int mode_of_op = cur.mode_of_operation_display[4];
        //printf("mode of operation display(4) = %d\n", mode_of_op);

        /* Check for master state (optional). */
		check_master_state();

		/* Check for slave configuration state(s) (optional). */
		check_slave_config_states();
		//for (i=0; i<2; i++) {
		//	printf("vel[%d]=%d\n", i, tar.target_velocity[i]);
		//}

		/*
		double odometry_constant=626594.7934;  // in ticks/m		
		double speed0 = cur.actual_velocity[0] / odometry_constant;
		double speed1 = cur.actual_velocity[1] / odometry_constant;
		double pos0 = cur.position[0] / odometry_constant;
		double pos1 = cur.position[1] / odometry_constant;
			
		
		//printf("0: Pos=%d  Vel=%d   1: Pos=%d Vel=%d\n", cur.position[0], cur.actual_velocity[0],  cur.position[1], cur.actual_velocity[1]);
		printf("0: Pos=%8.3f  Vel=%4.3f   1: Pos=%8.3f  Vel=%4.3f\n", pos0, speed0, pos1, speed1);
		*/
		
        //Ugly hack!  FIXME: Move this to function that deals with the drive operational state machine (is separate from the EtherCAT comm state machine)
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

        for (i = 0; i< NUM_DRIVES; i++) {
			int controlword = 0x00;
			int statusword = cur.status[i];	
			
			//printf("StatusWord[%i] = 0x%04x = %d \n", i, statusword, statusword);
			if (statusword & (1<<STATUSWORD_FAULT_BIT)) {
			   printf("\e[31;1mm%d Has fault! <----------------\e[0m\n", i);
			   }
		
			   if (!(statusword & (1<<STATUSWORD_VOLTAGE_ENABLE_BIT))){
				   printf("\e[31;1mm%d Voltage not enabled! <----------------\e[0m\n", i);
			   }

			   //if (!(statusword & (1 << STATUSWORD_SWITCH_ON_DISABLED_BIT))){
			//	   printf("\e[31;1mm%d Switch_on_disabled_bit <----------------\e[0m\n", i);
			  //}
		
		
			   if (!(statusword & (1<<STATUSWORD_OPERATION_ENABLE_BIT))) {
				   if (!(statusword & (1<<STATUSWORD_SWITCHED_ON_BIT))) {
					   if (!(statusword & (1<<STATUSWORD_READY_TO_SWITCH_ON_BIT))) {
						   if ((statusword & (1<<STATUSWORD_FAULT_BIT))) {
						   /* reset fault */
							controlword = 	0x80;
							printf("\e[31;1mm%d Reset fault <----------------\e[0m\n", i);
			   				//EC_WRITE_U16(domain1_pd + off_controlword[i], controlword);
						   } else {
							   /* shutdown */
							   controlword = 0x06;
							   printf("\e[31;1mm%d Shutdown <----------------\e[0m\n", i);
						   }
					   } else {
						/* switch on */
					   	controlword = 0x07;
					  	printf("\e[31;1mm%d Switch on <----------------\e[0m\n", i);
					   }
				   } else {
					   /* enable operation */
					   printf("\e[31;mm%d EnableOperation <----------------\e[0m\n", i);
					   controlword = 0x0F;
				   }
			   } else {
				// All is good, continue
				controlword = 0x0f;
			   }
		   
		   
			   EC_WRITE_U16(domain1_pd + off_controlword[i], controlword);
		
		
		   
		}   
		
		
		
		
	}

	/*
	if(tar.target_position[0] == 0 && 
	   tar.target_position[1] == 0) { 
		for (i = 0; i < 2; i++) { 
			EC_WRITE_U32(domain1_pd + off_target_position[i], cur.position[i]);
		}
		prevent_set_position = 1; 
	} else { 
		prevent_set_position = 0; 
	}
	*/

	/* Write process data. */
	/* Note: You _must_ write something, as this is how the drives sync. */

    //only feeding the wheel drives target velocity
    for (i = 0; i < 4; i++) {
        tar.profile_acceleration[i] = 5000000;
        tar.profile_deceleration[i] = 5000001;

        EC_WRITE_S32(domain1_pd + off_target_velocity[i], tar.target_velocity[i]  );
        EC_WRITE_U32(domain1_pd + off_profile_velocity[i], tar.profile_velocity[i]);
        //EC_WRITE_U16(domain1_pd + off_controlword[i], controlword);  //We are dealing with the control word before
        EC_WRITE_U32(domain1_pd + off_profile_acceleration[i], tar.profile_acceleration[i]);    // 5000000
        EC_WRITE_U32(domain1_pd + off_profile_deceleration[i], tar.profile_deceleration[i]);	//2000000 was smoothing out the jumpiness before

	}


	/* Send process data. */
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);

	cur.pkg_count = counter;

}


/*****************************************************************************/


static void enforce_max_velocities(omniwrite_t *t)
{
	int i;

    for (i = 0; i < 4; i++) {
		t->target_velocity[i] =
		  (t->target_velocity[i] > max_v) ? max_v : t->target_velocity[i];
		t->target_velocity[i] =
		  (t->target_velocity[i] < -max_v) ? -max_v : t->target_velocity[i];
	}
}


/*****************************************************************************/

static void stop_motors(void)
{
	int i;

    for (i = 0; i < NUM_DRIVES; i++)
        EC_WRITE_S32(domain1_pd + off_target_velocity[i], 0);

	/* Send process data. */
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);

	//usleep(1000);
}


static void timespecInc(struct timespec *tick, int nsec)
{
  tick->tv_nsec += nsec;
  while (tick->tv_nsec >= 1e9)
  {
    tick->tv_nsec -= 1e9;
    tick->tv_sec++;
  }
}


void* realtimeMain(void* udata)
{
  struct timespec tick;
  int period = 1e+6; // 1 ms in nanoseconds
  //int period = 5e+5; // 0.5 ms in nanoseconds

  // Initialize the tick struct.
  // TODO: Would it be better to do the following instead?
  //       clock_gettime(CLOCK_REALTIME, &before);
  tick.tv_sec = 0;
  tick.tv_nsec = 0;

  while(!exiting)
  {
    cyclic_task();

    if(pthread_mutex_trylock(&mutex) == 0)
    {
      tar = tar_buffer;
      cur_buffer = cur;
      pthread_mutex_unlock(&mutex);
    }

    // Compute end of next period
    timespecInc(&tick, period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + before.tv_nsec/1e9) > (tick.tv_sec + tick.tv_nsec/1e9))
    {
      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period) * period;
      timespecInc(&tick, period);

      misses++;
    }
    // Sleep until end of period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  }

  stop_motors();

  
  printf("Releasing master...\n");
  ecrt_release_master(master);

  return 0;
}


/*****************************************************************************/

/* Interface functions */

int start_omni_realtime(int max_vel)
{
	int i;

	max_v = max_vel;

	printf("Init omni...\n");

	/* Zero out the target/current data structs, just in case. */
	memset(&tar, 0, sizeof(tar));
	memset(&cur, 0, sizeof(cur));
	cur.magic_version = OMNICOM_MAGIC_VERSION;

	printf("Starting omni....\n");

	if (!(master = ecrt_request_master(0))) {
		printf( "Requesting master 0 failed!\n");
		goto out_return;
	}

	printf("Registering domain...\n");
	if (!(domain1 = ecrt_master_create_domain(master))) {
		printf( "Domain creation failed!\n");
		goto out_release_master;
	}

    for (i = 0; i < NUM_DRIVES; i++) {
        /* master, (slave alias, slave position), (VID, PID) */
		if (!(sc[i] = ecrt_master_slave_config(master, 0, i /* M?SlavePos */, ELMOG))) {
			printf(
			       "Failed to get slave configuration for motor %d.\n", i);
			goto out_release_master;
		}
        printf("Configuring PDOs for motor %d...\n", i);
		/* slave config, sync manager index, index of the PDO to assign */
        if (ecrt_slave_config_pdos(sc[i], EC_END, foo_syncs)) {
            printf( "Failed to configure PDOs for motor %d.\n", i);
            goto out_release_master;
        }
	}

	printf("Registering PDO entries...\n");
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
		printf( "PDO entry registration failed!\n");
		goto out_release_master;
	}

	printf("Activating master...\n");
	if (ecrt_master_activate(master)) {
		printf( "Failed to activate master!\n");
		goto out_release_master;
	}
	/* Get internal process data for domain. */
	domain1_pd = ecrt_domain_data(domain1);

	printf("Starting cyclic thread.\n");

    pthread_attr_t tattr;
    struct sched_param sparam;
    sparam.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_init(&tattr);
    pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
    pthread_attr_setschedparam(&tattr, &sparam);
    pthread_attr_setinheritsched (&tattr, PTHREAD_EXPLICIT_SCHED);
    
    if(pthread_create(&thread, &tattr, &realtimeMain, 0) != 0) {
      printf("# ERROR: could not create realtime thread\n");
      goto out_release_master;
    }


	printf("Started.\n");

	return 1;

out_release_master:
	printf( "Releasing master...\n");
	ecrt_release_master(master);
out_return:
	printf( "Failed to load. Aborting.\n");
	return 0;
}

/*****************************************************************************/

void stop_omni_realtime(void)
{
	printf("Stopping...\n");


	/* Signal a stop the realtime thread */
    exiting = 1;
    pthread_join(thread, 0);

	/* Now stop all motors. */
	//stop_motors();
	//usleep(100);


	printf("Unloading.\n");
}


void omni_write_data(struct omniwrite data)
{
  pthread_mutex_lock(&mutex);
  tar_buffer = data;
  enforce_max_velocities(&tar_buffer);
  pthread_mutex_unlock(&mutex);
}

struct omniread omni_read_data()
{
  struct omniread data;
  pthread_mutex_lock(&mutex);
  data = cur_buffer;
  pthread_mutex_unlock(&mutex);
  return data;
}

ec_master_t* get_master()
{
    return(master);
}
