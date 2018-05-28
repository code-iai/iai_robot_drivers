
#ifndef REALTIME_H
#define REALTIME_H

/* If you change the interface in any way, increase OMNICOM_MAGIC_VERSION */
#define OMNICOM_MAGIC_VERSION 1003
#define NUM_DRIVES 4

#include <ecrt.h>  //part of igh's ethercat master

/* Data we read from the EtherCAT slaves */
typedef struct omniread {  
	int16_t  magic_version; // magic number to prevent version clashes
	uint32_t pkg_count;     // Set in omnidrive kernel module

    int32_t position[NUM_DRIVES];
    uint32_t digital_inputs[NUM_DRIVES];
    int32_t actual_velocity[NUM_DRIVES];
    uint16_t status[NUM_DRIVES];
    int8_t mode_of_operation_display[NUM_DRIVES];
    int16_t actual_torque[NUM_DRIVES];


	// ethercat states
    int slave_state[NUM_DRIVES];
    int slave_online[NUM_DRIVES];
    int slave_operational[NUM_DRIVES];
	int master_link;
	int master_al_states;
	int master_slaves_responding;
	int working_counter;
	int working_counter_state;
} omniread_t;

/* Data we write to the EtherCAT slaves */
typedef struct omniwrite { 
	int16_t  magic_version;         // magic number to prevent version clashes
    int32_t target_position[NUM_DRIVES];
    int32_t target_velocity[NUM_DRIVES];
    int16_t target_torque[NUM_DRIVES];
    int16_t max_torque[NUM_DRIVES];
    uint16_t control_word[NUM_DRIVES];
    int8_t mode_of_operation[NUM_DRIVES];
    uint32_t profile_velocity[NUM_DRIVES];
    uint32_t profile_acceleration[NUM_DRIVES];
    uint32_t profile_deceleration[NUM_DRIVES];

} omniwrite_t;

// realtime interface

void omni_write_data(struct omniwrite data);
struct omniread omni_read_data();

int start_omni_realtime(int max_vel);
void stop_omni_realtime();

ec_master_t* get_master();


#endif // REALTIME_H
