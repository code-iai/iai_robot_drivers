/*
 * This file is part of the omni_ethercat project.
 *
 * Copyright (C) 2012-2016 Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de>
 * Copyright (C) 2009-2012 Ingo Kresse <kresse@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <soft_runstop/Handler.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
//#include <iai_control_msgs/PowerState.h>
#include <omni_ethercat/omnilib.hpp>   //library for the mecanum kinematics
#include <omni_ethercat/ecat_iface.hpp>  //library for interfacing with the Ethercat Motor drivers


//#include <sensor_msgs/JointState.h>

class Omnidrive
{
private:
	ros::NodeHandle n_;
	diagnostic_updater::Updater diagnostic_;
	//ros::Publisher current_pub_;
	//ros::Publisher power_pub_;
	//ros::Publisher js_pub_; //torso
	//ros::Subscriber power_sub_;
	ros::Time watchdog_time_;
	double drive_[3], drive_last_[3];
	//double torso_des_pos_; // torso
	//bool fresh_torso_des_pos_; //torso
	soft_runstop::Handler soft_runstop_handler_;
	std::string odom_frame_id_;
	std::string odom_child_frame_id_;
	//std::string power_name_;
	void cmdArrived(const geometry_msgs::TwistStamped::ConstPtr& msg);
	//void torsoCmdArrived(const std_msgs::Float64::ConstPtr& msg); //torso
	//void stateUpdate(diagnostic_updater::DiagnosticStatusWrapper &s);
	//void powerCommand(const iai_control_msgs::PowerState::ConstPtr& msg);

	//Variables to hold the desired twist
	omni_ethercat::Twist2d des_twist_;
	omni_ethercat::Twist2d limited_twist_;
	omni_ethercat::Twist2d max_twist_;
	double max_wheel_speed_;
	omni_ethercat::JacParams jac_params_;

public:
	Omnidrive();
	void main();
};

Omnidrive::Omnidrive() : n_("omnidrive"), diagnostic_(), soft_runstop_handler_(Duration(0.5))
{
	//diagnostic_.setHardwareID("omnidrive");
	//diagnostic_.add("Base", this, &Omnidrive::stateUpdate);
	n_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
	n_.param("odom_child_frame_id", odom_child_frame_id_, std::string("/base_footprint"));
	//n_.param("power_name", power_name_, std::string("Wheels"));


	//power_pub_ = n_.advertise<iai_control_msgs::PowerState>("/power_state", 1);
	//power_sub_ = n_.subscribe<iai_control_msgs::PowerState>("/power_command", 16, &Omnidrive::powerCommand, this);

	//js_pub_ = n_.advertise<sensor_msgs::JointState>("/torso/joint_states", 1);  //torso

	//initialize the twists to all zeroes
	des_twist_ = omni_ethercat::Twist2d(0,0,0);
	limited_twist_ = omni_ethercat::Twist2d(0.5,0,0);
	max_twist_ = omni_ethercat::Twist2d(1.0, 1.0, 1.0); //FIXME: read from param
	double max_wheel_tick_speed = 833333.0; // ticks/s : 5000 rpm/ 60s * 10000 ticks/rev
	double lx = 0.39225;
	double ly = 0.303495;
	double drive_constant = 626594.7934; // in ticks/m
	max_wheel_speed_ = max_wheel_tick_speed; //FIXME: read from param, specify units (rads/s?)
	jac_params_ = omni_ethercat::JacParams(lx, ly, drive_constant);  // lx, ly, drive-constant in ticks/m


	for(int i=0; i < 3; i++) {
	  drive_last_[i] = 0;
	  drive_[i] = 0;
	}

	watchdog_time_ = ros::Time::now();
}

void Omnidrive::cmdArrived(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

	//std::cout << "cmdArrived()" << std::endl;

	// FIXME: use TwistStamped instead of Twist and check that people command in the right frame
	// NOTE: No need for synchronization since this is called inside spinOnce() in the main loop

	//Checking that the frame is correct
	if (msg->header.frame_id == odom_child_frame_id_) {
		//The frame in the message matches the one of the base
		//read desired twist for the base
		des_twist_ = omni_ethercat::fromTwistMsg(msg->twist);
		//limit to our maximum values
		limited_twist_ = omni_ethercat::limitTwist(des_twist_, max_twist_);
		//limit again if any wheel exceeds the maximum wheel speed
		limited_twist_ = omni_ethercat::limitTwist(limited_twist_, jac_params_, max_wheel_speed_);

		if (des_twist_ != limited_twist_) {
			std::cout << "The desired twist will be limited from: " << des_twist_ << " to: " << limited_twist_ << std::endl;
		}
	
		//std::cout << "limited_twist_ = " << limited_twist_ << std::endl;

		watchdog_time_ = ros::Time::now();
	} else {
		std::cerr << "Twist command arrived expressed in a wrong frame.";
	}


}

//void Omnidrive::stateUpdate(diagnostic_updater::DiagnosticStatusWrapper &s)
//{
//  int estop;
//  char drive[5]; //+1 for torso
//  omnidrive_status(&drive[0], &drive[1], &drive[2], &drive[3], &drive[4], &estop);
//
//  bool operational = (drive[0] == '4' &&
//                      drive[1] == '4' &&
//                      drive[2] == '4' &&
//                      drive[3] == '4' &&
//                      drive[4] == '4' &&
//                      estop == 0);
//
//  if(operational)
//    s.summary(0, "Operational");
//  else
//    s.summary(1, "Down");
//  
//  for(int i=0; i < num_drives; i++)
//    s.add(std::string("status drive ") + (char) ('1' + i),
//          std::string("") + drive[i]);
//  s.add("Emergency Stop", (estop) ? "== pressed ==" : "released");
//
//  commstatus_t comm = omnidrive_commstatus();
//
//  for(int i=0; i < num_drives; i++)
//    s.addf(std::string("comm status drive ")+(char) ('1' + i),
////    s.addf("cstatus drive",
//           "0x%02X, %s, %s operational",
//             comm.slave_state[i],
//             comm.slave_online[i] ? "online" : "offline",
//             comm.slave_operational[i] ? "" : "not");
//
//  s.addf("master state", "Link is %s, %d slaves, AL states: 0x%02X",
//           comm.master_link ? "up" : "down",
//           comm.master_slaves_responding,
//           comm.master_al_states);
//
//  s.addf("working counter", "%d, %s",
//           comm.working_counter,
//           comm.working_counter_state == 2 ? "complete" : "incomplete");
//
//  iai_control_msgs::PowerState power;
//  power.name = power_name_;
//  power.enabled = operational;
//
//  power_pub_.publish(power);
//}


////FIXME: Do we need this for this base? This allows to bring the ethercat drives down and up at wish
//void Omnidrive::powerCommand(const iai_control_msgs::PowerState::ConstPtr& msg)
//{
//  if(msg->name == power_name_)
//  {
//    printf("Received power command!\n");
//    int estop;
//    char drive[4];
//    omnidrive_status(&drive[0], &drive[1], &drive[2], &drive[3], &drive[4], &estop);
//
//    bool power_state = (drive[0] == '4' &&
//                        drive[1] == '4' &&
//                        drive[2] == '4' &&
//                        drive[3] == '4' &&
//                        drive[4] == '4' &&
//                        estop == 0);
//                        
//    printf("Power_state=%d    drive[0]=%d\n", power_state, drive[0]);
//
//    if(msg->enabled == true && power_state == false)
//    {
//      printf("Recovering\n");
//      omnidrive_recover();
//      omnidrive_poweron();
//    }
//
//    if(msg->enabled == false && power_state == true)
//    {
//      printf("Turning off\n");
//      omnidrive_poweroff();
//    }
//    
//  }
//}

void Omnidrive::main()
{
	double speed, acc_max, t, radius, drift;
	int tf_frequency, runstop_frequency, js_frequency;
	const int loop_frequency = 250; // 250Hz update frequency

	//n_.param("speed", speed, 100.0); // 0.1
	// default acc: brake from max. speed to 0 within 1.5cm
	//n_.param("acceleration", acc_max, 1000.0);  //0.5*speed*speed/0.015

	n_.param("tf_frequency", tf_frequency, 50);
	n_.param("js_frequency", js_frequency, 125);
	n_.param("runstop_frequency", runstop_frequency, 10);
	n_.param("watchdog_period", t, 0.15);
	ros::Duration watchdog_period(t);
	n_.param("odometry_correction", drift, 1.0);



	omni_ecat::EcatAdmin ecat_admin;
	int init_worked = ecat_admin.ecat_init();

	if (init_worked != 0) {
		ROS_ERROR("failed to initialize the ethercat bus");
		ROS_ERROR("check dmesg and try \"sudo /etc/init.d/ethercat restart\"");
		return;
	}

	//tf::TransformBroadcaster transforms;

	ros::Subscriber sub = n_.subscribe("/base/cmd_vel", 10, &Omnidrive::cmdArrived, this);
	//ros::Subscriber sub_torso = n_.subscribe("/torso/cmd_vel", 10, &Omnidrive::torsoCmdArrived, this); //torso
	//ros::Publisher hard_runstop_pub = n_.advertise<std_msgs::Bool>("/hard_runstop", 1);

	//double x=0, y=0, a=0, torso_pos=0;

	int tf_publish_counter=0;
	int tf_send_rate = loop_frequency / tf_frequency;


	//torso:
	int js_publish_counter=0;
	int js_send_rate = loop_frequency / js_frequency;

	int runstop_publish_counter=0;
	int runstop_send_rate = loop_frequency / runstop_frequency;

	ros::Rate r(loop_frequency);
	ros::Time last_drive_check_time = ros::Time::now();

	while(n_.ok()) {

		// Check if we need to enable the drives once a second
		if ( ecat_admin.finished_ecat_init() && (ros::Time::now() - last_drive_check_time) > ros::Duration(0.1)) {
			last_drive_check_time = ros::Time::now();
			ecat_admin.check_drive_state();
		}

		//omnidrive_odometry(&x, &y, &a, &torso_pos);


		//FIXME: Do we need a watchdog for the torso?

		//The watchdog for the /cmd_vel topic
		//should stop the base if now new messages arrive
		if( (( ros::Time::now() - watchdog_time_) > watchdog_period) || soft_runstop_handler_.getState()) {

			//printf("Watchdog!\n");
			//printf("State of the soft_runstop = %d\n", soft_runstop_handler_.getState());


			//Only send the ROS warning the first time
			//when it had some driving velocities, and getting here not because of
			//the runstop
			if((drive_[0] != 0 || drive_[1] != 0 || drive_[2] !=0)
					&& !soft_runstop_handler_.getState())
				ROS_WARN("engaged watchdog!");

			//zero the command twist
			limited_twist_ = omni_ethercat::Twist2d(0.0, 0.0, 0.0);

			//While the watchdog is active, revisit here after watchdog_period
			watchdog_time_ = ros::Time::now();
		}


		//This is a Vector holding 4 velocities, in this order of wheels: fl, fr, bl, br.
		//The wheel axes are chosen such that if the base is moving forward as a whole, all of them have positive rotations
		// imagine all wheels with the rotational axis pointing to the left.
		omni_ethercat::OmniEncVel vels;
		vels = omni_ethercat::omniIK(jac_params_, limited_twist_);

		static omni_ethercat::Twist2d old_twist;
		static omni_ethercat::OmniEncVel old_vels;

		if (old_twist != limited_twist_) {
			old_twist = limited_twist_;
			std::cout << "Commanded twist: " << limited_twist_ << std::endl;
		}
		if (old_vels != vels) {
			old_vels = vels;
			std::cout << "Commanded wheel velocities: " << vels << std::endl;
		}

		
		//Actually command the wheel velocities
		ecat_admin.drive_map["fl"]->task_wdata_user_side.target_velocity = vels[0];
		ecat_admin.drive_map["fr"]->task_wdata_user_side.target_velocity = -1 * vels[1];
		ecat_admin.drive_map["bl"]->task_wdata_user_side.target_velocity = vels[2];
		ecat_admin.drive_map["br"]->task_wdata_user_side.target_velocity = -1 * vels[3];
		ecat_admin.drive_map["fl"]->task_wdata_user_side.profile_velocity = abs(vels[0]);
		ecat_admin.drive_map["fr"]->task_wdata_user_side.profile_velocity = abs(-1 * vels[1]);
		ecat_admin.drive_map["bl"]->task_wdata_user_side.profile_velocity = abs(vels[2]);
		ecat_admin.drive_map["br"]->task_wdata_user_side.profile_velocity = abs(-1 * vels[3]);

		//ecat_admin.drive_map["torso"]->task_wdata_user_side.target_velocity = 0;


		for (auto & drive_el: ecat_admin.drive_map) {
			auto & drive = drive_el.second;
			drive->task_wdata_user_side.profile_acceleration = 5000000;

			drive->task_wdata_user_side.profile_deceleration = 5000001;
		}


		//Evil acceleration limitation
		// this runs in a slow loop
		//FIXME: Move to the high-speed loop for smoothness -> Maybe not needed, this loop is 250Hz.
		//FIXME: Limit in twist-space, make it use time

		//    for(int i=0; i < 3; i++) {
		//      // acceleration limiting
		//      double acc = drive_[i] - drive_last_[i];
		//      double fac_rot = (i == 2) ? 1.0/radius : 1.0;
		//      acc = LIMIT(acc, acc_max*fac_rot*fac_rot);
		//      drive_[i] = drive_last_[i] + acc;
		//
		//      // velocity limiting
		//      drive_[i] = LIMIT(drive_[i], speed*fac_rot);
		//
		//      drive_last_[i] = drive_[i];
		//    }

		//if the watchdog was activated drive_[0-2] are 0.0
		//only call omnidrive_drive once in the loop
		//the last call will probably override the previous ones
		//    omnidrive_drive(drive_[0], drive_[1], drive_[2], torso_des_pos_);

		// publish odometry readings
		//    if(++tf_publish_counter == tf_send_rate) {
		//      tf::Quaternion q;
		//      q.setRPY(0, 0, a);
		//      tf::Transform pose(q, tf::Point(x, y, 0.0));
		//      // FIXME: publish this on a separate topic like /base/odom
		//      transforms.sendTransform(tf::StampedTransform(pose, ros::Time::now(), frame_id_, child_frame_id_));
		//      // FIXME: publish actual base twist on topic like /base/vel
		//      tf_publish_counter = 0;
		//    }
		//
		//
		//    // publish torso position
		//    if(++js_publish_counter == js_send_rate) {
		//      sensor_msgs::JointState msg;
		//      msg.header.stamp = ros::Time::now();
		//      msg.name.push_back("triangle_base_joint");
		//      msg.position.push_back(torso_pos);
		//      // FIXME: report the actual values
		//      msg.velocity.push_back(0.0);
		//      msg.effort.push_back(0.0);
		//      js_pub_.publish(msg);
		//      js_publish_counter = 0;
		//    }
		//
		//    // publish hard runstop state
		//    // FIXME: report real hard E-stop status
		//    // in Rosie, the hard runstop was read from the ethercat drives
		//    // in Boxy it is part of the state reported by the ELMO drives
		//    if(++runstop_publish_counter == runstop_send_rate) {
		//      int runstop=0;
		//      omnidrive_status(0,0,0,0,0, &runstop);
		//      std_msgs::Bool msg;
		//      msg.data = (runstop != 0);
		//      hard_runstop_pub.publish(msg);
		//      runstop_publish_counter = 0;
		//    }

		// process incoming messages
		ros::spinOnce();

		//diagnostic_.update();
		r.sleep();

	}

	ecat_admin.ec_drives_vel_zero();
	ecat_admin.shutdown();

}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "omni_ethercat");
	std::cout << "Starting up\n"  ;


	Omnidrive drive;
	drive.main();


}


