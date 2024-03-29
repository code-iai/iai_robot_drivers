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
#include <ecrt.h>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");


class Omnidrive {
private:
    ros::NodeHandle n_;
    diagnostic_updater::Updater diagnostic_;
    ros::Publisher js_pub_; //joint_states
    ros::Time watchdog_time_;
    ros::Time torso_watchdog_time_;

    soft_runstop::Handler soft_runstop_handler_;
    std::string odom_frame_id_;
    std::string odom_child_frame_id_;
    double jac_lx_param_, jac_ly_param_, drive_constant_param_, max_wheel_tick_speed_param_;
    double max_dx_param_, max_dy_param_, max_dtheta_param_;
    double js_frequency_param_, runstop_frequency_param_, watchdog_period_param_;
    bool torso_present_param_;
    double torso_ticks_to_m_param_;
    int cpu_core_;


    void twistStampedCommand(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void giskardCommand(const sensor_msgs::JointState &msg);
    void twistCommand(const geometry_msgs::Twist::ConstPtr& msg);



    //Variables to hold the desired twist
    omni_ethercat::Twist2d des_twist_;
    omni_ethercat::Twist2d limited_twist_;
    omni_ethercat::Twist2d max_twist_;
    double max_wheel_speed_;
    omni_ethercat::JacParams jac_params_;
    double des_torso_vel_;

    std::map<std::string, unsigned int> joint_name_to_index_;

    void diagnostic_state_update(diagnostic_updater::DiagnosticStatusWrapper &s);
    std::shared_ptr<omni_ecat::EcatAdmin> ecat_admin;



public:
    Omnidrive();

    void main();
};

Omnidrive::Omnidrive() : n_("omnidrive"), diagnostic_(n_), soft_runstop_handler_(Duration(0.5)) {
    diagnostic_.setHardwareID("omnidrive");
    diagnostic_.add("boxy_omni_v1", this, &Omnidrive::diagnostic_state_update);


    n_.param("js_frequency", js_frequency_param_, 125.0);
    n_.param("runstop_frequency", runstop_frequency_param_, 10.0);
    n_.param("watchdog_period", watchdog_period_param_, 0.15);

    n_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    n_.param("odom_child_frame_id", odom_child_frame_id_, std::string("/base_footprint"));
    n_.param("jac_lx",  jac_lx_param_, 0.30375 );
    n_.param("jac_ly",  jac_ly_param_, 0.39475 );

    // drive_constant = amount of ticks needed for a one meter translation (ticks/m)
    // calculated thus: 10000 (ticks/turn given by encoder) * gear ratio / ( pi * diameter)
    // diameter is 8" = 8*25.4/1000 = 0.797965m
    // circumference for that 8" wheel is = 0.638372m
    // For donbot: drive_constant = 10000*20/(3.14159*8*25.4/1000)
    //double drive_constant = 626594; // in ticks/m  For Boxy (gear ratio 40:1)
    //for donbot: adjusting the 0.4% error in translation when moving
    n_.param("drive_constant",  drive_constant_param_,
             10000 * 20 / (3.14159 * 8 * 1.004 * 25.4 / 1000)); // in ticks/m
    n_.param("max_wheel_tick_speed", max_wheel_tick_speed_param_, 5000.0 / 60.0 * 10000.0 ); // ticks/s: 5000rpm / 60s * 10k ticks/rev

    n_.param("torso_ticks_to_m", torso_ticks_to_m_param_, 10000000.0); //calculated with the 10k ticks/rev and the slope of the ball screw

    n_.param("max_dx", max_dx_param_, 1.0 ); // in m/s
    n_.param("max_dy", max_dy_param_, 1.0 ); // in m/s
    n_.param("max_dtheta", max_dtheta_param_, 3.14159 / 4.0 ); // in rads/s
    n_.param("cpu_core", cpu_core_, 10 ); // cpu number

    n_.param("torso_present", torso_present_param_, false); // Does the base have a torso as the 5th controller?

    std::string odom_x_joint_name_param_, odom_y_joint_name_param_, odom_z_joint_name_param_;
    n_.param("odom_x_joint_name", odom_x_joint_name_param_, std::string("odom_x_joint"));
    n_.param("odom_y_joint_name", odom_y_joint_name_param_, std::string("odom_y_joint"));
    n_.param("odom_z_joint_name", odom_z_joint_name_param_, std::string("odom_z_joint"));

    //Only looking for the info on these joint from the giskard message
    joint_name_to_index_.insert(std::pair<std::string, unsigned int>(odom_x_joint_name_param_, 0));
    joint_name_to_index_.insert(std::pair<std::string, unsigned int>(odom_y_joint_name_param_, 1));
    joint_name_to_index_.insert(std::pair<std::string, unsigned int>(odom_z_joint_name_param_, 2));

    if (torso_present_param_) {
        joint_name_to_index_.insert(std::pair<std::string, unsigned int>("triangle_base_joint", 3));
    }

    //Report to console the used parameters, will help catch configuration errors
    ROS_INFO("Base node starting. The following parameters in namespace %s configure this node:", n_.getNamespace().c_str());
    ROS_INFO("param: %s = %f", "js_frequency", js_frequency_param_);
    ROS_INFO("param: %s = %f", "runstop_frequency", runstop_frequency_param_);
    ROS_INFO("param: %s = %f", "watchdog_period", watchdog_period_param_);
    ROS_INFO("param: %s = %i", "cpu_core", cpu_core_);
    ROS_INFO("param: %s = \"%s\"", "odom_frame_id", odom_frame_id_.c_str());
    ROS_INFO("param: %s = \"%s\"", "odom_child_frame_id", odom_child_frame_id_.c_str());
    ROS_INFO("param: %s = %f", "jac_lx", jac_lx_param_);
    ROS_INFO("param: %s = %f", "jac_ly", jac_ly_param_);
    ROS_INFO("param: %s = %f", "drive_constant", drive_constant_param_);
    ROS_INFO("param: %s = %f", "max_wheel_tick_speed", max_wheel_tick_speed_param_);
    ROS_INFO("param: %s = %f", "max_dx", max_dx_param_);
    ROS_INFO("param: %s = %f", "max_dy", max_dx_param_);
    ROS_INFO("param: %s = %f", "max_dtheta", max_dtheta_param_);
    ROS_INFO("param: %s = \"%s\"", "odom_x_joint_name", odom_x_joint_name_param_.c_str());
    ROS_INFO("param: %s = \"%s\"", "odom_y_joint_name", odom_y_joint_name_param_.c_str());
    ROS_INFO("param: %s = \"%s\"", "odom_z_joint_name", odom_z_joint_name_param_.c_str());


    ecat_admin = std::make_shared<omni_ecat::EcatAdmin>(torso_present_param_, cpu_core_);

    //main odometry output using joint_states: needs support in the URDF, to have odom_x_joint, odom_y_joint, odom_z_joint
    js_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);

    //initialize the twists to all zeroes
    des_twist_ = omni_ethercat::Twist2d(0, 0, 0);
    limited_twist_ = omni_ethercat::Twist2d(0, 0, 0);
    des_torso_vel_ = 0.0;

    max_twist_ = omni_ethercat::Twist2d(max_dx_param_, max_dy_param_, max_dtheta_param_);

    max_wheel_speed_ = max_wheel_tick_speed_param_;
    jac_params_ = omni_ethercat::JacParams(jac_lx_param_, jac_ly_param_, drive_constant_param_);  // lx, ly, drive-constant in ticks/m

    watchdog_time_ = ros::Time::now();

}

//publish to diagnostics
void Omnidrive::diagnostic_state_update(diagnostic_updater::DiagnosticStatusWrapper &s)
{
    s.summary(diagnostic_msgs::DiagnosticStatus::OK, "Controller running");

    bool hard_run_stop = ecat_admin->get_global_sto_state();
    s.add("Hardware Run-Stop", (hard_run_stop) ? "released" : "== pressed ==" );

    bool soft_run_stop = soft_runstop_handler_.getState();
    s.add("Soft Run-Stop", (soft_run_stop) ?  "== pressed ==" : "released");

    bool zero_goal_vel = (des_twist_.norm() < 0.0001);
    s.add("Received goal velocities equal to zero", (zero_goal_vel) ? "true" : "false" );

    s.addf("master state", "Link is %s, %d slaves, AL states: 0x%02X",
           ecat_admin->master_state.link_up ? "up" : "down",
           ecat_admin->master_state.slaves_responding,
           ecat_admin->master_state.al_states);

    bool all_drives_happy = true;

    for (auto &drive_el: ecat_admin->drive_map) {
        auto &name = drive_el.first;
        auto &drive = drive_el.second;

        s.addf(std::string("drive ['") + std::string(name) + std::string("'] state"), "Drive is: %s, operational: %s, AL state: 0x%02X",
               drive->slave_config_state.online ? "online" : "offline",
               drive->slave_config_state.operational ? "true" : "false",
               drive->slave_config_state.al_state);

        all_drives_happy = all_drives_happy and drive->slave_config_state.online;

        s.addf(std::string("drive ['") + std::string(name) + std::string("'] bus voltage"), "%f", drive->task_rdata_user_side.dc_link_voltage / 1000.0);
    }

    if (not all_drives_happy) {
        s.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "At least one drive is not operational");
    }

}


//receive Twist commands for legacy software
void Omnidrive::twistCommand(const geometry_msgs::Twist::ConstPtr& msg)
{

    //just make a message of type TwistStamped and pass it to our other function
    boost::shared_ptr<geometry_msgs::TwistStamped> ts_msg_ptr(new geometry_msgs::TwistStamped);

    //give it the right frame of reference
    ts_msg_ptr->header.frame_id = odom_child_frame_id_;

    ts_msg_ptr->twist.linear = msg->linear;
    ts_msg_ptr->twist.angular = msg->angular;

    twistStampedCommand(ts_msg_ptr);

}


void Omnidrive::twistStampedCommand(const geometry_msgs::TwistStamped::ConstPtr &msg) {

    //std::cout << "twistStampedCommand()" << std::endl;

    // NOTE: No need for synchronization since the ros callbacks are called
    // serially by calling spinOnce() in the main loop

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
            ROS_WARN_STREAM_THROTTLE(1.0, "The desired twist will be limited from: " << des_twist_.format(CommaInitFmt) << " to: "
                                                                                     << limited_twist_.format(CommaInitFmt));
        }

        if ( (msg->twist.linear.z != 0.0) or (msg->twist.angular.x != 0.0) or (msg->twist.angular.y != 0.0)) {
            ROS_WARN_STREAM_THROTTLE(1.0, "The desired twist includes velocities that can't be executed (transz, rotx, roty). They will be ignored.");

        }

        //pet the watchdog
        watchdog_time_ = ros::Time::now();

    } else {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Twist command arrived expressed in a wrong frame, and will be ignored. The right frame is: " << odom_child_frame_id_);

    }


}

void Omnidrive::giskardCommand(const sensor_msgs::JointState &msg) {

    bool received_one_valid_command = false;
    bool received_torso_command = false;

    //check length of arrays, if they don't match, bail out
    unsigned long len_name = msg.name.size();
    unsigned long len_vel = msg.velocity.size();

    if (len_name != len_vel) {
        ROS_WARN_ONCE(
                "Omnidrive::giskardCommand: Received wrong length of arrays. Ignoring. This warning will only print once.");
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
                    //command for odom_y_joint
                    des_twist_[1] = msg.velocity[i];
                    received_one_valid_command = true;
                } else if (j == 2) { //odom_z_joint
                    //command for odom_z_joint
                    des_twist_[2] = msg.velocity[i];
                    received_one_valid_command = true;
                } else if (j == 3) {
                    //received a command for the torso
                    des_torso_vel_ = msg.velocity[i];
                    received_torso_command = true;
                    //ROS_INFO("got a torso command");
                }


            }

        }


        if (received_one_valid_command) {
            //limit to our maximum values
            limited_twist_ = omni_ethercat::limitTwist(des_twist_, max_twist_);
            //limit again if any wheel exceeds the maximum wheel speed
            limited_twist_ = omni_ethercat::limitTwist(limited_twist_, jac_params_, max_wheel_speed_);


            if (des_twist_ != limited_twist_) {
                ROS_WARN_STREAM_THROTTLE(1.0, "The desired twist will be limited from: " << des_twist_.format(CommaInitFmt) << " to: "
                                                                                         << limited_twist_.format(CommaInitFmt));
            }

            //pet the watchdog
            watchdog_time_ = ros::Time::now();

        }

        if (received_torso_command) {

            torso_watchdog_time_ = ros::Time::now();
        }
    }

}


void Omnidrive::main() {
    //reading ros parameters in Omnidrive's constructor
    const double loop_frequency = 250.0; // 250Hz update frequency for the main loop

    ros::Duration watchdog_period(watchdog_period_param_);

    int ecat_init_worked = ecat_admin->ecat_init();

    if (ecat_init_worked != 0) {
        ROS_ERROR("failed to initialize the ethercat bus");
        ROS_ERROR("check dmesg and try \"sudo /etc/init.d/ethercat restart\"");
        return;
    }


    ros::Subscriber sub_twist_stamped = n_.subscribe("cmd_vel", 1, &Omnidrive::twistStampedCommand, this);
    ros::Subscriber sub_twist = n_.subscribe("cmd_vel_twist", 1, &Omnidrive::twistCommand, this);
    ros::Subscriber sub_giskard = n_.subscribe("giskard_command", 1, &Omnidrive::giskardCommand, this);
    ros::Publisher hard_runstop_pub = n_.advertise<std_msgs::Bool>("/hard_runstop", 1);


    //joint_state publisher
    unsigned int js_publish_counter = 0;
    unsigned int js_send_rate = (unsigned int)(loop_frequency / js_frequency_param_);

    unsigned int runstop_publish_counter = 0;
    unsigned int runstop_send_rate = (unsigned int)(loop_frequency / runstop_frequency_param_);

    ros::Rate r(loop_frequency);
    ros::Time last_drive_check_time = ros::Time::now();

    //Initialize encoder data to current one
    omni_ethercat::OmniEncPos old_encoder_data(
            {double(ecat_admin->drive_map["fl"]->task_rdata_user_side.actual_position),
             double(ecat_admin->drive_map["fr"]->task_rdata_user_side.actual_position),
             double(ecat_admin->drive_map["bl"]->task_rdata_user_side.actual_position),
             double(ecat_admin->drive_map["fr"]->task_rdata_user_side.actual_position)});

    //Initialize start odometry to zero (the convention is that it starts in zero where the base turns on).
    omni_ethercat::Pose2d current_odometry({0.0, 0.0, 0.0});

    //FIXME: Consider moving these to the constructor, bad if they are not set before we use the rest
    ecat_admin->jac_params_ = jac_params_;
    ecat_admin->torso_ticks_to_m_ = torso_ticks_to_m_param_;

    while (n_.ok()) {

        // Check if we need to re-enable the drives at 10Hz
        if (ecat_admin->finished_ecat_init() and (ros::Time::now() - last_drive_check_time) > ros::Duration(0.1)) {
            last_drive_check_time = ros::Time::now();
            ecat_admin->check_drive_state();
        }


        //Calculate odometry
        ros::Time time_of_odom = ros::Time::now();
        omni_ethercat::OmniEncPos current_encoder_data;
        omni_ethercat::OmniEncVel current_speed_data_ticks;
        omni_ethercat::Twist2d current_speed_base, current_speed_odom;

        current_encoder_data = {double(ecat_admin->drive_map["fl"]->task_rdata_user_side.actual_position),
                                double(ecat_admin->drive_map["fr"]->task_rdata_user_side.actual_position),
                                double(ecat_admin->drive_map["bl"]->task_rdata_user_side.actual_position),
                                double(ecat_admin->drive_map["br"]->task_rdata_user_side.actual_position)};

        current_speed_data_ticks = {double(ecat_admin->drive_map["fl"]->task_rdata_user_side.actual_velocity),
                                    double(ecat_admin->drive_map["fr"]->task_rdata_user_side.actual_velocity),
                                    double(ecat_admin->drive_map["bl"]->task_rdata_user_side.actual_velocity),
                                    double(ecat_admin->drive_map["br"]->task_rdata_user_side.actual_velocity)};

        //FIXME: figure out how to convert actual_torque to a decent unit (amps, NM, ...)
        Eigen::Vector4d currents = {double(ecat_admin->drive_map["fl"]->task_rdata_user_side.actual_torque),
                                    double(ecat_admin->drive_map["fr"]->task_rdata_user_side.actual_torque),
                                    double(ecat_admin->drive_map["bl"]->task_rdata_user_side.actual_torque),
                                    double(ecat_admin->drive_map["br"]->task_rdata_user_side.actual_torque)};

        //Current cartesian speed in the base reference frame
        current_speed_base = omni_ethercat::omniFK(jac_params_, current_speed_data_ticks);
        //Convert the speed to the odom frame of reference
        current_speed_odom = omni_ethercat::changeReferenceFrame(current_odometry, current_speed_base);


        //Integrate the change of position in the wheels into the odometry
        omni_ethercat::OmniEncVel encoder_diff = current_encoder_data - old_encoder_data;
        current_odometry = omni_ethercat::nextOdometry(current_odometry, encoder_diff, jac_params_);

        //std::cout << "odometry: " << current_odometry[0] <<" " << current_odometry[1] << " " <<current_odometry[2] << std::endl;
        //std::cout << "wheel pos: " << current_encoder_data[0] << " " << current_encoder_data[1] << " " << current_encoder_data[2] << " " << current_encoder_data[3] << " " << std::endl;
        //std::cout << "encoder diff: " << double(encoder_diff[0]) << " " << encoder_diff[1] << " " << encoder_diff[2] << " " << encoder_diff[3] << " " << std::endl;
        //save the current encoders for calculating the delta next time
        old_encoder_data = current_encoder_data;


        //The watchdog for the commanding topics
        //should stop the base if no new messages arrive
        if (((ros::Time::now() - watchdog_time_) > watchdog_period) || soft_runstop_handler_.getState()) {

            //printf("Watchdog!\n");
            //printf("State of the soft_runstop = %d\n", soft_runstop_handler_.getState());


            if (!soft_runstop_handler_.getState() and
                !(des_twist_[0] == 0.0 and des_twist_[1] == 0.0 and des_twist_[2] == 0.0))
                ROS_WARN_THROTTLE(1, "engaged watchdog!");

            //zero the command twist
            des_twist_ = omni_ethercat::Twist2d(0.0, 0.0, 0.0);
            limited_twist_ = omni_ethercat::Twist2d(0.0, 0.0, 0.0);

            //While the watchdog is active, revisit here after watchdog_period
            watchdog_time_ = ros::Time::now();
        }

        //Watchdog for the torso
        if (((ros::Time::now() - torso_watchdog_time_) > watchdog_period) || soft_runstop_handler_.getState()) {
            //zero the torso velocity
            des_torso_vel_ = 0.0;
            torso_watchdog_time_ = ros::Time::now();
        }


        static omni_ethercat::Twist2d old_twist;

        //print if there is a new commanded velocity
        if ( (old_twist != limited_twist_) and ecat_admin->get_global_sto_state() ){
            old_twist = limited_twist_;
            //ROS_INFO_STREAM("Commanded twist: " << limited_twist_.format(CommaInitFmt));
            //Tell the interpolator our new goal twist
            ecat_admin->set_new_goal_twist(limited_twist_[0], limited_twist_[1], limited_twist_[2]);
        }

        if (torso_present_param_ and ecat_admin->get_global_sto_state() ) {
            ecat_admin->set_new_torso_goal_vel(des_torso_vel_);  //FIXME: Consider only setting this when the goal changes
        }


        // publish odometry readings
        if (++js_publish_counter == js_send_rate) {

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

            if (torso_present_param_) {
                msg.name.push_back("triangle_base_joint");
                double torso_pos = double(ecat_admin->drive_map["torso"]->task_rdata_user_side.actual_position) / torso_ticks_to_m_param_;
                msg.position.push_back(torso_pos);
                msg.velocity.push_back(0.0);
                msg.effort.push_back(0.0);
            }


            js_pub_.publish(msg);
            js_publish_counter = 0;
        }


            // publish hard runstop state
            if(++runstop_publish_counter == runstop_send_rate) {
              std_msgs::Bool msg;
              msg.data = ecat_admin->get_global_sto_state(); //true means that the drives are free to run. False is that the e-stop is active
              hard_runstop_pub.publish(msg);
              runstop_publish_counter = 0;
            }

        // process incoming messages
        ros::spinOnce();

        diagnostic_.update();
        r.sleep();

    }

    //When stopping the node, make sure the velocities are zero and the drives get shutdown
    ecat_admin->ec_drives_vel_zero();
    ecat_admin->shutdown();

}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "omni_ethercat");
    ROS_INFO("Starting up");


    Omnidrive drive;
    drive.main();
}


