/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "iai_wsg_50_msgs/Status.h"
#include "iai_wsg_50_msgs/Move.h"
#include "iai_wsg_50_msgs/Conf.h"
#include "iai_wsg_50_msgs/Incr.h"
#include "iai_wsg_50_msgs/Cmd.h"
#include "iai_wsg_50_msgs/PositionCmd.h"
#include "iai_wsg_50_msgs/SpeedCmd.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

float increment;
bool objectGraspped;

int g_timer_cnt = 0;
ros::Publisher g_pub_state, g_pub_joint, g_pub_moving;
bool g_ismoving = false;
float g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
   
std::string l_finger_joint_name, r_finger_joint_name, reference_frame;
//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

bool setAccSrv(iai_wsg_50_msgs::Conf::Request &req, iai_wsg_50_msgs::Conf::Response &res)
{
	setAcceleration(req.val);
	return true;
}

bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ack_fault();
	return true;
}

/** \brief Callback for goal_position topic (in appropriate modes) */
void position_cb(const iai_wsg_50_msgs::PositionCmd::ConstPtr& msg)
{
    g_speed = msg->speed; 
    g_goal_position = msg->pos;
    setGraspingForceLimit(msg->force);
    // timer_cb() will send command to gripper
}

/** \brief Callback for goal_speed topic (in appropriate modes) */
void speed_cb(const iai_wsg_50_msgs::SpeedCmd::ConstPtr& msg)
{
    g_goal_speed = msg->speed;
    g_speed = msg->speed;
    setGraspingForceLimit(msg->force);
    // timer_cb() will send command to gripper
}

/** \brief Loop for state polling in modes script and polling. Also sends command in script mode. */
void timer_cb(const ros::TimerEvent& ev)
{
	// ==== Get state values by built-in commands ====
	gripper_response info;
	float acc = 0.0;
	info.speed = 0.0;
  
	// ==== Call custom measure-and-move command ====
	int res = 0;
	if (!isnan(g_goal_position)) {
		ROS_INFO("Position command: pos=%5.1f, speed=%5.1f", g_goal_position, g_speed);
        res = script_measure_move(1, g_goal_position, g_speed, info);
	} else if (!isnan(g_goal_speed)) {
		ROS_INFO("Velocity command: speed=%5.1f", g_goal_speed);
        res = script_measure_move(2, 0, g_goal_speed, info);
	} else
        res = script_measure_move(0, 0, 0, info);
	if (!isnan(g_goal_position))
		g_goal_position = NAN;
	if (!isnan(g_goal_speed))
		g_goal_speed = NAN;

	if (!res) {
		ROS_ERROR("Measure-and-move command failed");
		return;
	}

	// ==== Moving msg ====
	if (g_ismoving != info.ismoving) {
		std_msgs::Bool moving_msg;
		moving_msg.data = info.ismoving;
		g_pub_moving.publish(moving_msg);
		g_ismoving = info.ismoving;
	}

	// ==== Status msg ====
	iai_wsg_50_msgs::Status status_msg;
	status_msg.status = info.state_text;
	status_msg.width = info.position;
	status_msg.speed = info.speed;
	status_msg.acc = acc;
	status_msg.force = info.f_motor;
	status_msg.force_finger0 = info.f_finger0;
	status_msg.force_finger1 = info.f_finger1;

	g_pub_state.publish(status_msg);
             

	// ==== Joint state msg ====
	sensor_msgs::JointState joint_states;
	joint_states.header.stamp = ros::Time::now();;
	joint_states.header.frame_id = reference_frame;
	joint_states.name.push_back(l_finger_joint_name);
	joint_states.name.push_back(r_finger_joint_name);
	
	joint_states.position.resize(2);

	joint_states.position[0] = -info.position/2000.0;
	joint_states.position[1] = info.position/2000.0;
	joint_states.velocity.resize(2);		
    joint_states.velocity[0] = info.speed/1000.0;
    joint_states.velocity[1] = info.speed/1000.0;
	joint_states.effort.resize(2);
	joint_states.effort[0] = info.f_motor;
	joint_states.effort[1] = info.f_motor;
	
	g_pub_joint.publish(joint_states);

	// printf("Timer, last duration: %6.1f\n", ev.profile.last_duration.toSec() * 1000.0);
}


void sigint_handler(int sig) {
    ROS_INFO("Exiting...");
    ros::shutdown();
}

/**
 * The main function
 */

int main( int argc, char **argv )
{
   ros::init(argc, argv, "wsg_50");
   ros::NodeHandle nh("~");
   signal(SIGINT, sigint_handler);

   std::string ip, protocol;
   int port, local_port;
   double rate, grasping_force;
   bool use_udp = false;

   nh.param("ip", ip, std::string("192.168.1.20"));
   nh.param("port", port, 1000);
   nh.param("local_port", local_port, 1501);
   nh.param("protocol", protocol, std::string(""));
   nh.param("rate", rate, 1.0); // With custom script, up to 30Hz are possible
   nh.param("grasping_force", grasping_force, 0.0);
   nh.param("l_finger_joint_name", l_finger_joint_name, std::string("wsg_50_gripper_base_joint_gripper_left"));
   nh.param("r_finger_joint_name", r_finger_joint_name, std::string("wsg_50_gripper_base_joint_gripper_right"));
   nh.param("reference_frame", reference_frame, std::string("wsg_50_gripper_base_link"));
	
   if (protocol == "udp")
       use_udp = true;
   else
       protocol = "tcp";

   ROS_INFO("Connecting to %s:%d (%s); ...", ip.c_str(), port, protocol.c_str());

   // Connect to device using TCP/USP
   int res_con;
   if (!use_udp)
       res_con = cmd_connect_tcp( ip.c_str(), port );
   else
       res_con = cmd_connect_udp(local_port, ip.c_str(), port );

   if (res_con == 0 ) {
        ROS_INFO("Gripper connection stablished");

		// Services
        ros::ServiceServer ackSS, setAccSS;

        ackSS = nh.advertiseService("ack", ackSrv);
        setAccSS = nh.advertiseService("set_acceleration", setAccSrv);
    

		// Subscriber
        ros::Subscriber sub_position, sub_speed;
        sub_position = nh.subscribe("goal_position", 5, position_cb);
        sub_speed = nh.subscribe("goal_speed", 5, speed_cb);

		// Publisher
		g_pub_state = nh.advertise<iai_wsg_50_msgs::Status>("status", 1000);
		g_pub_joint = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        g_pub_moving = nh.advertise<std_msgs::Bool>("moving", 10);

		ROS_INFO("Ready to use, homing now...");
		homing();

		if (grasping_force > 0.0) {
			ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
			setGraspingForceLimit(grasping_force);
		}

        ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
        std::thread th;
        ros::Timer tmr;
        tmr = nh.createTimer(ros::Duration(1.0/rate), timer_cb);

        ros::spin();

	} else {
        ROS_ERROR("Unable to connect, please check the port and address used.");
	}

   ROS_INFO("Exiting...");
   sleep(1);
   cmd_disconnect();

	return 0;

}


//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
