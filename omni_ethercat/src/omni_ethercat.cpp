/*
 * Copyright (C) 2009 by Ingo Kresse <kresse@in.tum.de>
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


#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <soft_runstop/Handler.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <iai_control_msgs/PowerState.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>



//For the torso:
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>


extern "C" {
#include "omnilib.h"
}

#define LIMIT(x, l) ( (x>l) ? l : (x<-l) ? -l : x )

const int num_drives = 5;
const double torso_ticks_to_m = 10000000;


//FIXME: param that says which ethercat drives are what
//example: drives 0-3 are on wheels, counterclockwise, starting on front left.
//example: plus drive 4 is the torso

class Omnidrive
{
private:
  ros::NodeHandle n_;
  diagnostic_updater::Updater diagnostic_;
  ros::Publisher current_pub_;
  ros::Publisher power_pub_;
  ros::Publisher js_pub_; //torso
  ros::Subscriber power_sub_;
  ros::Publisher odom_pub_;
  ros::Time watchdog_time_;
  ros::Time watchdog_torso_time_;

  double drive_[3], drive_last_[3];
  double torso_des_vel_; // torso
  soft_runstop::Handler soft_runstop_handler_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string power_name_;
  void cmdArrived(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void cmdArrivedTwist(const geometry_msgs::Twist::ConstPtr& msg);
  void torsoCmdArrived(const std_msgs::Float64::ConstPtr& msg); //torso
  void giskardCommand(const sensor_msgs::JointState& msg);
  bool giskardCommand_reported_error;
  void stateUpdate(diagnostic_updater::DiagnosticStatusWrapper &s);
  void powerCommand(const iai_control_msgs::PowerState::ConstPtr& msg);

  std::map<std::string, unsigned int> joint_name_to_index_;
public:
  Omnidrive();
  void main();
};


Omnidrive::Omnidrive() : n_("omnidrive"), diagnostic_(), soft_runstop_handler_(Duration(0.5))
{
  diagnostic_.setHardwareID("omnidrive");
  diagnostic_.add("Base", this, &Omnidrive::stateUpdate);
  n_.param("frame_id", frame_id_, std::string("odom"));
  n_.param("child_frame_id", child_frame_id_, std::string("base_link"));
  n_.param("power_name", power_name_, std::string("Wheels"));

  //current_pub_ = n_.advertise<std_msgs::Float64MultiArray>("motor_currents", 1);
  power_pub_ = n_.advertise<iai_control_msgs::PowerState>("/power_state", 1);
  power_sub_ = n_.subscribe<iai_control_msgs::PowerState>("/power_command", 16, &Omnidrive::powerCommand, this);

  odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 3);

  js_pub_ = n_.advertise<sensor_msgs::JointState>("/torso/joint_states", 1);  //torso

  for(int i=0; i < 3; i++) {
    drive_last_[i] = 0;
    drive_[i] = 0;
  }


  //Only looking for the info on one joint from the giskard message
  joint_name_to_index_.insert(std::pair<std::string, unsigned int>("triangle_base_joint",0));
  giskardCommand_reported_error = false;


  watchdog_time_ = ros::Time::now();
}

void Omnidrive::cmdArrivedTwist(const geometry_msgs::Twist::ConstPtr& msg)
{

	if ( std::isnan(msg->linear.x) or std::isnan(msg->linear.y) or std::isnan(msg->angular.z)) {
		drive_[0] = 0;
		drive_[1] = 0;
		drive_[2] = 0;
		ROS_WARN("command got nan, dropping it");
	} else {
		drive_[0] = msg->linear.x;
		drive_[1] = msg->linear.y;
		drive_[2] = msg->angular.z;
	}

	//FIXME:  FUGLY, this needs to be fixed in omnilib.c (fix the jacobian)
	//  plus, the direction of rotation and position of wheels needs to be documented
	//Rotate 180deg around the z axis
	drive_[0] = -drive_[0];
	drive_[1] = -drive_[1];

	//also the z rotation was wrong
	drive_[2] = -drive_[2];


	watchdog_time_ = ros::Time::now();

}

void Omnidrive::cmdArrived(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  // FIXME: use TwistStamped instead of Twist and check that people command in the right frame
  // NOTE: No need for synchronization since this is called inside spinOnce() in the main loop

  drive_[0] = msg->twist.linear.x;
  drive_[1] = msg->twist.linear.y;
  drive_[2] = msg->twist.angular.z;


  //FIXME:  FUGLY, this needs to be fixed in omnilib.c (fix the jacobian)
  //  plus, the direction of rotation and position of wheels needs to be documented
  //Rotate 180deg around the z axis
  drive_[0] = -drive_[0];
  drive_[1] = -drive_[1];

  //also the z rotation was wrong
  drive_[2] = -drive_[2];


  watchdog_time_ = ros::Time::now();
}

//torso:
//FIXME: need param to turn torso into velocity or position resolved
//FIXME: All topics receiving commands need a watchdog, this one too
 
void Omnidrive::torsoCmdArrived(const std_msgs::Float64::ConstPtr& msg)
{

  printf("Desired torso velocity: %f m/s\n", msg->data);
  torso_des_vel_ = msg->data * torso_ticks_to_m;
  watchdog_torso_time_ = ros::Time::now();
}

void Omnidrive::giskardCommand(const sensor_msgs::JointState& msg)
{

  bool received_one_valid_command = false;

  //check length of arrays, if they don't match, bail out
  int len_name = msg.name.size();
  int len_vel = msg.velocity.size();

  if (len_name != len_vel) {
      if (not giskardCommand_reported_error) {
          ROS_WARN("Omnidrive::giskardCommand: Received wrong length of arrays. Ignoring. This warning will only print once.");
          giskardCommand_reported_error = true;
      }
      return;
  }

  //Search the names for a matching name in the list of joints controlled by this process
  for (unsigned int i=0; i < len_name ; ++i) {
	  std::map<std::string, unsigned int>::iterator it;
	  it = joint_name_to_index_.find(msg.name[i]); //returns an iterator if found, otherwise map::end

	  if (it != joint_name_to_index_.end()) {
		  //the name is in my list
		  unsigned int j = it->second;
		  torso_des_vel_ = msg.velocity[i] * torso_ticks_to_m;
		  //std::cout << "Assigned " << it->first << " value " << torso_des_vel_ << std::endl;
		  received_one_valid_command = true;
	  }

  }


  if (received_one_valid_command) {
	  //pet the watchdog
	  watchdog_torso_time_ = ros::Time::now();

  }


}



void Omnidrive::stateUpdate(diagnostic_updater::DiagnosticStatusWrapper &s)
{
  int estop;
  char drive[5]; //+1 for torso
  omnidrive_status(&drive[0], &drive[1], &drive[2], &drive[3], &drive[4], &estop);

  bool operational = (drive[0] == '4' &&
                      drive[1] == '4' &&
                      drive[2] == '4' &&
                      drive[3] == '4' &&
                      drive[4] == '4' &&
                      estop == 0);

  if(operational)
    s.summary(0, "Operational");
  else
    s.summary(1, "Down");
  
  for(int i=0; i < num_drives; i++)
    s.add(std::string("status drive ") + (char) ('1' + i),
          std::string("") + drive[i]);
  s.add("Emergency Stop", (estop) ? "== pressed ==" : "released");

  commstatus_t comm = omnidrive_commstatus();

  for(int i=0; i < num_drives; i++)
    s.addf(std::string("comm status drive ")+(char) ('1' + i),
//    s.addf("cstatus drive",
           "0x%02X, %s, %s operational",
             comm.slave_state[i],
             comm.slave_online[i] ? "online" : "offline",
             comm.slave_operational[i] ? "" : "not");

  s.addf("master state", "Link is %s, %d slaves, AL states: 0x%02X",
           comm.master_link ? "up" : "down",
           comm.master_slaves_responding,
           comm.master_al_states);

  s.addf("working counter", "%d, %s",
           comm.working_counter,
           comm.working_counter_state == 2 ? "complete" : "incomplete");

  iai_control_msgs::PowerState power;
  power.name = power_name_;
  power.enabled = operational;

  power_pub_.publish(power);
}


//FIXME: Do we need this for this base? This allows to bring the ethercat drives down and up at wish
void Omnidrive::powerCommand(const iai_control_msgs::PowerState::ConstPtr& msg)
{
  if(msg->name == power_name_)
  {
    printf("Received power command!\n");
    int estop;
    char drive[4];
    omnidrive_status(&drive[0], &drive[1], &drive[2], &drive[3], &drive[4], &estop);

    bool power_state = (drive[0] == '4' &&
                        drive[1] == '4' &&
                        drive[2] == '4' &&
                        drive[3] == '4' &&
                        drive[4] == '4' &&
                        estop == 0);
                        
    printf("Power_state=%d    drive[0]=%d\n", power_state, drive[0]);

    if(msg->enabled == true && power_state == false)
    {
      printf("Recovering\n");
      omnidrive_recover();
      omnidrive_poweron();
    }

    if(msg->enabled == false && power_state == true)
    {
      printf("Turning off\n");
      omnidrive_poweroff();
    }
    
  }
}

void Omnidrive::main()
{
  double speed, acc_max, t, radius, drift;
  int runstop_frequency, js_frequency;
  const int loop_frequency = 250; // 250Hz update frequency

  n_.param("speed", speed, 100.0); // 0.1
  // default acc: brake from max. speed to 0 within 1.5cm
  n_.param("acceleration", acc_max, 1000.0);  //0.5*speed*speed/0.015
  // radius of the robot
  n_.param("radius", radius, 0.6);
  n_.param("js_frequency", js_frequency, 125);
  n_.param("runstop_frequency", runstop_frequency, 10);
  n_.param("watchdog_period", t, 0.15);
  ros::Duration watchdog_period(t);
  n_.param("odometry_correction", drift, 1.0);

  // set acceleration to correct scale
  acc_max /= loop_frequency;

  if(omnidrive_init() != 0) {
    ROS_ERROR("failed to initialize omnidrive");
    ROS_ERROR("check dmesg and try \"sudo /etc/init.d/ethercat restart\"");
    return;
  }
  omnidrive_set_correction(drift);

  ros::Subscriber sub = n_.subscribe("/base/cmd_vel", 10, &Omnidrive::cmdArrived, this);
  ros::Subscriber sub_twist = n_.subscribe("/base/cmd_vel_twist", 10, &Omnidrive::cmdArrivedTwist, this);
  ros::Subscriber sub_torso = n_.subscribe("/torso/cmd_vel", 10, &Omnidrive::torsoCmdArrived, this); //torso
  ros::Subscriber sub_giskard = n_.subscribe("giskard_command", 1, &Omnidrive::giskardCommand, this); //torso too
  ros::Publisher hard_runstop_pub = n_.advertise<std_msgs::Bool>("/hard_runstop", 1);

  double x=0, y=0, a=0, torso_pos=0;

  //torso and odometry published as joint_states:
  int js_publish_counter=0;
  int js_send_rate = loop_frequency / js_frequency;

  int runstop_publish_counter=0;
  int runstop_send_rate = loop_frequency / runstop_frequency;

  ros::Rate r(loop_frequency);

  while(n_.ok()) {

    omnidrive_odometry(&x, &y, &a, &torso_pos);


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

        //zero the velocities of the wheels
        for(int i=0; i < 3; i++) {
            drive_last_[i] = 0;
            drive_[i] = 0;
        }

        //While the watchdog is active, revisit here after watchdog_period
        watchdog_time_ = ros::Time::now();
    }

    if( (( ros::Time::now() - watchdog_torso_time_) > watchdog_period) || soft_runstop_handler_.getState()) {
    	//ROS_WARN("torso_watchdog active");
    	//Watchdog for the torso
    	torso_des_vel_ = 0.0;
    	//While the watchdog is active, revisit here after watchdog_period
    	watchdog_torso_time_ = ros::Time::now();

    }

    //Evil acceleration limitation
    // this runs in a slow loop
    //FIXME: Move to the high-speed loop for smoothness -> Maybe not needed, this loop is 250Hz.
    //FIXME: Limit in twist-space, make it use time 
    
    for(int i=0; i < 3; i++) {
      // acceleration limiting
      double acc = drive_[i] - drive_last_[i];
      double fac_rot = (i == 2) ? 1.0/radius : 1.0;
      acc = LIMIT(acc, acc_max*fac_rot*fac_rot);
      drive_[i] = drive_last_[i] + acc;

      // velocity limiting
      drive_[i] = LIMIT(drive_[i], speed*fac_rot);

      drive_last_[i] = drive_[i];
    }
   
    //if the watchdog was activated drive_[0-2] are 0.0
    //only call omnidrive_drive once in the loop
    //the last call will probably override the previous ones 
    omnidrive_drive(drive_[0], drive_[1], drive_[2], torso_des_vel_);



    // publish odometry readings
    // publish torso position
    if(++js_publish_counter == js_send_rate) {

      // FIXME: report the actual velocity and effort values

      sensor_msgs::JointState msg;
      msg.header.stamp = ros::Time::now();
      msg.name.push_back("triangle_base_joint");
      msg.position.push_back(torso_pos);
      msg.velocity.push_back(0.0);
      msg.effort.push_back(0.0);

      msg.name.push_back("odom_x_joint");
      msg.position.push_back(x);
      msg.velocity.push_back(0.0);
      msg.effort.push_back(0.0);

      msg.name.push_back("odom_y_joint");
      msg.position.push_back(y);
      msg.velocity.push_back(0.0);
      msg.effort.push_back(0.0);

      msg.name.push_back("odom_z_joint");
      msg.position.push_back(a);
      msg.velocity.push_back(0.0);
      msg.effort.push_back(0.0);

      js_pub_.publish(msg);
      js_publish_counter = 0;
    }

    // publish hard runstop state
    // FIXME: report real hard E-stop status
    // in Rosie, the hard runstop was read from the ethercat drives
    // in Boxy it is part of the state reported by the ELMO drives
    if(++runstop_publish_counter == runstop_send_rate) {
      int runstop=0;
      omnidrive_status(0,0,0,0,0, &runstop);
      std_msgs::Bool msg;
      msg.data = (runstop != 0);
      hard_runstop_pub.publish(msg);
      runstop_publish_counter = 0;
    }

    // process incoming messages
    ros::spinOnce();

    diagnostic_.update();
    r.sleep();
  }

  omnidrive_shutdown();

}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "omni_ethercat");

  Omnidrive drive;
  drive.main();
}
