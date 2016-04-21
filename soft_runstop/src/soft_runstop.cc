/*
 * Copyright (c) 2010, Ingo Kresse <kresse@in.tum.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technical University Munich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

using namespace ros;

class SoftRunstop
{
public:
  SoftRunstop();
  void main();
private:
  NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  bool stateMachine(bool stop_button, bool start_button, bool timeout);
  void joystickData(const sensor_msgs::JoyConstPtr& msg);
  int stop_button_, start_button_;

  bool running_;
  ros::Time t_msg_, t_guard_;
  ros::Duration guard_time_, timeout_;
  enum Runstate {
    RUNNING = 0,
    STOPPING = 1,
    GUARDING = 2,
    READY = 3,
    STARTING = 4
  };
  Runstate state_;
};


SoftRunstop::SoftRunstop() : n_("~"), running_(false), t_msg_(0,1), t_guard_(0,1), state_(STOPPING)
{
  double t;
  n_.param("guard_time", t, 1.5);
  guard_time_ = ros::Duration(t);
  n_.param("timeout", t, 0.5);
  timeout_ = ros::Duration(t);
  n_.param("stop_button", stop_button_, 0);
  n_.param("start_button", start_button_, 3);

  pub_ = n_.advertise<std_msgs::Bool>("/soft_runstop", 1);
  sub_ = n_.subscribe("/joy", 20, &SoftRunstop::joystickData, this);
}


void SoftRunstop::joystickData(const sensor_msgs::JoyConstPtr& msg)
{
  t_msg_ = ros::Time::now();

  bool stop = false;
  bool start = false;
  if(  (int) msg->buttons.size() >= stop_button_
    && (int) msg->buttons.size() >= start_button_)
  {
    stop = msg->buttons[stop_button_];
    start = msg->buttons[start_button_];
  }

  running_ = stateMachine(stop, start, false);
}


bool SoftRunstop::stateMachine(bool stop_button, bool start_button, bool timeout)
{
  if(timeout)
  {
    state_ = STOPPING;
    return false;
  }

  switch(state_)
  {
    case RUNNING:
      if(stop_button)
        state_ = STOPPING;
      break;
    case STOPPING:
      if(!stop_button)
      {
        t_guard_ = ros::Time::now();
        state_ = GUARDING;
      }
      break;
    case GUARDING:
      if(stop_button)
        state_ = STOPPING;
      else if(ros::Time::now() - t_guard_ > guard_time_)
        state_ = READY;
      break;
    case READY:
      if(start_button)
        state_ = STARTING;
      break;
    case STARTING:
      if(!start_button)
        state_ = RUNNING;
      break;
    default:
      state_ = STOPPING;
      break;
  }
  return (state_ == RUNNING);
}


void SoftRunstop::main()
{
  ros::Rate loop_rate(20);
  std_msgs::Bool runstop;

  while (ros::ok())
  {
    if(ros::Time::now() - t_msg_ > timeout_) {
      if(running_)
        ROS_INFO("Runstop Timeout");
      running_ = stateMachine(false, false, true);
    }

    runstop.data = !running_;
    pub_.publish(runstop);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // exiting, stop the robot
  runstop.data = true;

  pub_.publish(runstop);
  pub_.publish(runstop);
  pub_.publish(runstop);

  ros::spinOnce();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "soft_runstop");

  SoftRunstop s;
  s.main();

  return 0;
}
