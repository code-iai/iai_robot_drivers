/*
 * This file is part of the omni_ethercat project.
 *
 * Copyright (C) 2012-2018 Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de>
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
#include <soft_runstop/Handler.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <omni_ethercat/omnilib.hpp>   //library for the mecanum kinematics
#include <omni_ethercat/ecat_iface.hpp>  //library for interfacing with the Ethercat Motor drivers
#include <sensor_msgs/JointState.h>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");


class Omnidrive
{
private:
	ros::NodeHandle n_;
	diagnostic_updater::Updater diagnostic_;
	ros::Publisher js_pub_; //joint_states
	ros::Time watchdog_time_;
	soft_runstop::Handler soft_runstop_handler_;
	std::string odom_frame_id_;
	std::string odom_child_frame_id_;
	void cmdArrived(const geometry_msgs::TwistStamped::ConstPtr& msg);

	//Variables to hold the desired twist
	omni_ethercat::Twist2d des_twist_;
	omni_ethercat::Twist2d limited_twist_;
	omni_ethercat::Twist2d max_twist_;
	double max_wheel_speed_;
	omni_ethercat::JacParams jac_params_;

    std::map<std::string, unsigned int> joint_name_to_index_;
    void giskardCommand(const sensor_msgs::JointState& msg);


public:
	Omnidrive();
	void main();
};

Omnidrive::Omnidrive() : n_("omnidrive"), diagnostic_(), soft_runstop_handler_(Duration(0.5))
{
	//diagnostic_.setHardwareID("omnidrive");
	//diagnostic_.add("Base", this, &Omnidrive::stateUpdate); //FIXME: populate and publish diagnostics
	n_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
	n_.param("odom_child_frame_id", odom_child_frame_id_, std::string("/base_footprint"));

	js_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);

	//initialize the twists to all zeroes
	des_twist_ = omni_ethercat::Twist2d(0,0,0);
	limited_twist_ = omni_ethercat::Twist2d(0.5,0,0);
	max_twist_ = omni_ethercat::Twist2d(1.0, 1.0, 1.0); //FIXME: read from param
	double max_wheel_tick_speed = 833333.0; // ticks/s : 5000 rpm/ 60s * 10000 ticks/rev
	double lx = 0.39225;
	double ly = 0.303495;
	//FIXME: read these settings from the parameter server
	// drive_constant = amount of ticks needed for a one meter translation (ticks/m)
        // calculated thus: 10000 (ticks/turn given by encoder) * gear ratio / ( pi * diameter)
        // diameter is 8" = 8*25.4/1000 = 0.797965m
        // circumference for that 8" wheel is = 0.638372m
        // For donbot: drive_constant = 10000*20/(3.14159*8*25.4/1000)
	//double drive_constant = 626594; // in ticks/m  For Boxy (gear ratio 40:1)
        //for donbot: adjusting the 0.5% error in translation when moving
        double drive_constant = 10000*20/(3.14159*8*1.005*25.4/1000); // in ticks/m 


	max_wheel_speed_ = max_wheel_tick_speed; //FIXME: read from param, specify units (rads/s?)
	jac_params_ = omni_ethercat::JacParams(lx, ly, drive_constant);  // lx, ly, drive-constant in ticks/m

    //Only looking for the info on these joint from the giskard message
    joint_name_to_index_.insert(std::pair<std::string, unsigned int>("odom_x_joint",0));
    joint_name_to_index_.insert(std::pair<std::string, unsigned int>("odom_y_joint",1));
    joint_name_to_index_.insert(std::pair<std::string, unsigned int>("odom_z_joint",2));

	watchdog_time_ = ros::Time::now();

}

void Omnidrive::cmdArrived(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

	//std::cout << "cmdArrived()" << std::endl;

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
			ROS_WARN_STREAM_THROTTLE(0.5, "The desired twist will be limited from: " << des_twist_ << " to: " << limited_twist_);
		}



		watchdog_time_ = ros::Time::now();
	} else {
		ROS_ERROR_THROTTLE(0.5, "Twist command arrived expressed in a wrong frame.");

	}


}

void Omnidrive::giskardCommand(const sensor_msgs::JointState& msg)
{

    bool received_one_valid_command = false;

    //check length of arrays, if they don't match, bail out
    unsigned long len_name = msg.name.size();
    unsigned long len_vel = msg.velocity.size();

    if (len_name != len_vel) {
            ROS_WARN_ONCE("Omnidrive::giskardCommand: Received wrong length of arrays. Ignoring. This warning will only print once.");
        } else {

        //Search the names for a matching name in the list of joints controlled by this process
        for (unsigned int i = 0; i < len_name; ++i) {
            std::map<std::string, unsigned int>::iterator it;

            it = joint_name_to_index_.find(msg.name[i]); //returns an iterator if found, otherwise map::end

            if (it != joint_name_to_index_.end()) {
                //the name is in my list
                unsigned int j = it->second;

                if (j == 0) { //odom_x_joint
                    //command for odom_x_joint
                    des_twist_[0] = msg.velocity[i];
                    received_one_valid_command = true;
                } else if (j == 1) {  //odom_y_joint
                    //command for odom_x_joint
                    des_twist_[1] = msg.velocity[i];
                    received_one_valid_command = true;
                } else if (j == 2) { //odom_z_joint
                    //command for odom_x_joint
                    des_twist_[2] = msg.velocity[i];
                    received_one_valid_command = true;
                }


            }

        }


        if (received_one_valid_command) {
            //limit to our maximum values
            limited_twist_ = omni_ethercat::limitTwist(des_twist_, max_twist_);
            //limit again if any wheel exceeds the maximum wheel speed
            limited_twist_ = omni_ethercat::limitTwist(limited_twist_, jac_params_, max_wheel_speed_);

            if (des_twist_ != limited_twist_) {
                ROS_WARN_STREAM_THROTTLE(0.5, "The desired twist will be limited from: " << des_twist_ << " to: " << limited_twist_);
            }

            //pet the watchdog
            watchdog_time_ = ros::Time::now();

        }
    }

}


void Omnidrive::main()
{
    double watchdog_period_param;
	int tf_frequency, runstop_frequency, js_frequency;
	const int loop_frequency = 250; // 250Hz update frequency

	n_.param("tf_frequency", tf_frequency, 50);
	n_.param("js_frequency", js_frequency, 125);
	n_.param("runstop_frequency", runstop_frequency, 10);
	n_.param("watchdog_period", watchdog_period_param, 0.15);
	ros::Duration watchdog_period(watchdog_period_param);



	omni_ecat::EcatAdmin ecat_admin;
	int init_worked = ecat_admin.ecat_init();

	if (init_worked != 0) {
		ROS_ERROR("failed to initialize the ethercat bus");
		ROS_ERROR("check dmesg and try \"sudo /etc/init.d/ethercat restart\"");
		return;
	}

	//tf::TransformBroadcaster transforms;

	ros::Subscriber sub = n_.subscribe("cmd_vel", 10, &Omnidrive::cmdArrived, this);
	//ros::Publisher hard_runstop_pub = n_.advertise<std_msgs::Bool>("/hard_runstop", 1);
    ros::Subscriber sub_giskard = n_.subscribe("giskard_command", 1, &Omnidrive::giskardCommand, this);
	int tf_publish_counter = 0;
	int tf_send_rate = loop_frequency / tf_frequency;


	//joint_state publisher
	int js_publish_counter=0;
	int js_send_rate = loop_frequency / js_frequency;

	int runstop_publish_counter=0;
	int runstop_send_rate = loop_frequency / runstop_frequency;

	ros::Rate r(loop_frequency);
	ros::Time last_drive_check_time = ros::Time::now();

	//Initialize encoder data to current one
	omni_ethercat::OmniEncPos old_encoder_data({double(ecat_admin.drive_map["fl"]->task_rdata_user_side.actual_position),
                                                double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_position),
                                                double(ecat_admin.drive_map["bl"]->task_rdata_user_side.actual_position),
                                                double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_position)});

	//Initialize start odometry to zero (the convention is that it starts in zero where the base turns on).
    omni_ethercat::Pose2d current_odometry({0.0, 0.0, 0.0});


	while(n_.ok()) {

		// Check if we need to re-enable the drives at 10Hz
		if ( ecat_admin.finished_ecat_init() and (ros::Time::now() - last_drive_check_time) > ros::Duration(0.1)) {
			last_drive_check_time = ros::Time::now();
			ecat_admin.check_drive_state();
		}


		//Calculate odometry
		ros::Time time_of_odom = ros::Time::now();
		omni_ethercat::OmniEncPos current_encoder_data;
		omni_ethercat::OmniEncVel current_speed_data_ticks;
        omni_ethercat::Twist2d current_speed_base, current_speed_odom;

		current_encoder_data = {double(ecat_admin.drive_map["fl"]->task_rdata_user_side.actual_position),
                                double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_position),
                                double(ecat_admin.drive_map["bl"]->task_rdata_user_side.actual_position),
                                double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_position)};

		current_speed_data_ticks = {double(ecat_admin.drive_map["fl"]->task_rdata_user_side.actual_velocity),
                              double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_velocity),
                              double(ecat_admin.drive_map["bl"]->task_rdata_user_side.actual_velocity),
                              double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_velocity)};

		//FIXME: figure out how to convert actual_torque to a decent unit (amps, NM, ...)
		Eigen::Vector4d currents = {double(ecat_admin.drive_map["fl"]->task_rdata_user_side.actual_torque),
                                    double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_torque),
                                    double(ecat_admin.drive_map["bl"]->task_rdata_user_side.actual_torque),
                                    double(ecat_admin.drive_map["fr"]->task_rdata_user_side.actual_torque)};

		//Current cartesian speed in the base reference frame
		current_speed_base = omni_ethercat::omniFK(jac_params_, current_speed_data_ticks);
		//Convert the speed to the odom frame of reference
		current_speed_odom = omni_ethercat::changeReferenceFrame(current_odometry, current_speed_base);


        //Integrate the change of position in the wheels into the odometry
		auto encoder_diff = current_encoder_data - old_encoder_data;
		current_odometry = omni_ethercat::nextOdometry(current_odometry, encoder_diff, jac_params_);

        //std::cout << "odometry: " << current_odometry[0] <<" " << current_odometry[1] << " " <<current_odometry[2] << std::endl;
        //std::cout << "wheel pos: " << current_encoder_data[0] << " " << current_encoder_data[1] << " " << current_encoder_data[2] << " " << current_encoder_data[3] << " " << std::endl;
        //std::cout << "encoder diff: " << double(encoder_diff[0]) << " " << encoder_diff[1] << " " << encoder_diff[2] << " " << encoder_diff[3] << " " << std::endl;
        //save the current encoders for calculating the delta next time
        old_encoder_data = current_encoder_data;


		//The watchdog for the commanding topics
		//should stop the base if no new messages arrive
		if ( (( ros::Time::now() - watchdog_time_) > watchdog_period) || soft_runstop_handler_.getState() ) {

			//printf("Watchdog!\n");
			//printf("State of the soft_runstop = %d\n", soft_runstop_handler_.getState());


			if (!soft_runstop_handler_.getState() and !(des_twist_[0] == 0.0 and des_twist_[1] == 0.0 and des_twist_[2] == 0.0) )
				ROS_WARN_THROTTLE(1, "engaged watchdog!");

			//zero the command twist
            des_twist_ = omni_ethercat::Twist2d(0.0, 0.0, 0.0);
			limited_twist_ = omni_ethercat::Twist2d(0.0, 0.0, 0.0);

			//While the watchdog is active, revisit here after watchdog_period
			watchdog_time_ = ros::Time::now();
		}

		//Tell the interpolator our new goal twist
        ecat_admin.set_new_goal_twist(limited_twist_[0], limited_twist_[1], limited_twist_[2]);
		ecat_admin.jac_params_ = jac_params_;

		static omni_ethercat::Twist2d old_twist;

		//print if there is a new commanded velocity
		if (old_twist != limited_twist_) {
			old_twist = limited_twist_;
			ROS_INFO_STREAM("Commanded twist: " << limited_twist_.format(CommaInitFmt));
		}


        // publish odometry readings
        if(++js_publish_counter == js_send_rate) {

            // FIXME: report the actual velocity and effort values
            sensor_msgs::JointState msg;
            msg.header.stamp = time_of_odom;

            msg.name.push_back("odom_x_joint");
            msg.position.push_back(current_odometry[0]);
            msg.velocity.push_back(current_speed_odom[0]);
            msg.effort.push_back(0.0);

            msg.name.push_back("odom_y_joint");
            msg.position.push_back(current_odometry[1]);
            msg.velocity.push_back(current_speed_odom[1]);
            msg.effort.push_back(0.0);

            msg.name.push_back("odom_z_joint");
            msg.position.push_back(current_odometry[2]);
            msg.velocity.push_back(current_speed_odom[2]);
            msg.effort.push_back(0.0);

            js_pub_.publish(msg);
            js_publish_counter = 0;
        }


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

	//When stopping the node, make sure the velocities are zero and the drives get shutdown
	ecat_admin.ec_drives_vel_zero();
	ecat_admin.shutdown();

}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "omni_ethercat");
	ROS_INFO("Starting up");


    Omnidrive drive;
	drive.main();
}


