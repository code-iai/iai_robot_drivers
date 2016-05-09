/*
 * Copyright (c) 2015, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iai_kms_40_driver/kms_40_driver_node.hpp>
#include <iai_kms_40_driver/msg_conversions.hpp>
#include <iai_kms_40_driver/parser.hpp>

namespace iai_kms_40_driver
{
  struct timeval convertTime(double seconds)
  {
    ros::Duration tmp(seconds);

    // specify a timeout of 1 second
    struct timeval result;
    result.tv_sec = tmp.sec;
    result.tv_usec = tmp.nsec / 1000;

    return result;
  }

  KMS40DriverNode::KMS40DriverNode(const ros::NodeHandle& nh) : nh_(nh) 
  {
    pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
    tare_service_ = nh_.advertiseService("set_tare", &KMS40DriverNode::tare_service_callback, this);
    tare_service_ = nh_.advertiseService("set_hz", &KMS40DriverNode::hz_service_callback, this);
  }
  
  KMS40DriverNode::~KMS40DriverNode()
  { 
  }

  bool KMS40DriverNode::hz_service_callback(iai_kms_40_msgs::SetHz::Request& req, iai_kms_40_msgs::SetHz::Response& res){
    if (req.Hz == 0){
      driver_.publicKmsServiceRequest("LDIV()\n", "\n");
      return true;
    }
    bool result = false;
    std::ostringstream response, request;
    int frame_divider = 500/req.Hz;
    // std::cout << req.Hz << std::endl;
    // std::cout << frame_divider << std::endl;
    request << "LDIV(" << frame_divider << ")\n";
    response << "LDIV=" << frame_divider << "\n";    
    result = driver_.publicKmsServiceRequest(request.str(), response.str());
    res.success = result;
    return result; 
  }

  bool KMS40DriverNode::tare_service_callback(iai_kms_40_msgs::SetTare::Request& req, iai_kms_40_msgs::SetTare::Response& res)
  {
    bool result = false;
    if (req.tare){
      result = driver_.publicKmsServiceRequest("TARE(1)\n","TARE=1\n");
    } else {
      result = driver_.publicKmsServiceRequest("TARE(0)\n","TARE=0\n");
    }
    res.success = result;
    return result;
  }
  
  void KMS40DriverNode::run()
  {
    if(!startUp())
      return;
  
    loop();
  
    driver_.stop();
  }
  
  bool KMS40DriverNode::startUp()
  {
    std::string ip, port;
    double timeout;
    int frame_divider;

    if ( !nh_.getParam("ip", ip) )
    {
      ROS_ERROR("Could not find ROS parameter for IP");
      return false;
    }

    if ( !nh_.getParam("port", port) )
    {
      ROS_ERROR("Could not find ROS parameter for TCP port");
      return false;
    }

    if ( !nh_.getParam("tcp_timeout", timeout) )
    {
      ROS_ERROR("Could not find ROS parameter for TCP timeout");
      return false;
    }

    if ( !nh_.getParam("frame_id", msg_.header.frame_id) )
    {
      ROS_ERROR("Could not find ROS parameter for frame-id");
      return false;
    }

    if ( !nh_.getParam("publish_rate", frame_divider) )
    {
      ROS_ERROR("Could not find ROS parameter for frame divider");
      return false;
    } else {
      frame_divider = 500/frame_divider;
    }

    if ( frame_divider > 500 || frame_divider < 1)
    {
      ROS_ERROR("Frame divider not set to at least 1 or maximum 500");
      return false;
    }

    return driver_.start(ip, port, convertTime(timeout), frame_divider);
  }
  
  void KMS40DriverNode::loop()
  {
    int publish_rate;

    if ( !nh_.getParam("publish_rate", publish_rate) )
    {
      ROS_ERROR("Could not find ROS parameter for ROS publish rate");
      return;
    }

    if ( !(publish_rate > 0) )
    {
      ROS_ERROR("Given ROS publish rate was not greater 0");
      return;
    }

    ros::Rate r(publish_rate);
    while(ros::ok())
    {
      // TODO: Getting the timestamp here is a hack! This should come from the driver.
      //       However, at the time of writing the firmware only reported a meaningless
      //       timestamp, i.e. relative to system bootup time.
      msg_.header.stamp = ros::Time::now();
 
      pub_.publish(populateMsg(driver_.currentWrench(), msg_));
      ros::spinOnce();
      r.sleep();
    }
  }
} // namespace iai_kms_40_driver
