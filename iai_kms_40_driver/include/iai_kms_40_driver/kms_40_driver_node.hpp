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

#ifndef IAI_KMS_40_DRIVER_KMS_40_DRIVER_NODE_HPP_
#define IAI_KMS_40_DRIVER_KMS_40_DRIVER_NODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iai_kms_40_msgs/SetTare.h>
#include <iai_kms_40_msgs/SetHz.h>
#include <iai_kms_40_driver/kms_40_driver.hpp>

namespace iai_kms_40_driver
{
  class KMS40DriverNode
  {
    public:
      KMS40DriverNode(const ros::NodeHandle& nh); 
  
      ~KMS40DriverNode();
  
      void run();
  
    private:
      ros::NodeHandle nh_;
      ros::Publisher pub_;
      ros::ServiceServer tare_service_;
      geometry_msgs::WrenchStamped msg_;
      KMS40Driver driver_;
  
      bool startUp();
      void loop();
      bool tare_service_callback(iai_kms_40_msgs::SetTare::Request& req, iai_kms_40_msgs::SetTare::Response& res);
  };
} // namespace iai_kms_40_driver
#endif // IAI_KMS_40_DRIVER_KMS_40_DRIVER_NODE_HPP_
