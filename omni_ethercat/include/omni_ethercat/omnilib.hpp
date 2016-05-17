/*
 * This file is part of the libomnidrive project.
 *
 * Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#ifndef OMNI_ETHERCAT_OMNILIB_HPP
#define OMNI_ETHERCAT_OMNILIB_HPP

#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

namespace omni_ethercat
{
  typedef Eigen::Matrix< double, 3, 4 > OmniJac;
  typedef Eigen::Matrix< double, 4, 3 > OmniJacInv;
  typedef Eigen::Vector3d Pose2d;
  typedef Eigen::Vector3d Twist2d;
  typedef Eigen::Vector4d OmniEncPos;
  typedef Eigen::Vector4d OmniEncVel;

  inline geometry_msgs::Pose toPoseMsg(const Pose2d& pose)
  {
    geometry_msgs::Pose msg;
    msg.position.x = pose(0);
    msg.position.y = pose(1);
    msg.orientation = tf::createQuaternionMsgFromYaw(pose(2));
    return msg;
  }

  inline Pose2d fromPoseMsg(const geometry_msgs::Pose& msg)
  {
    Pose2d pose;
    pose(0) = msg.position.x;
    pose(1) = msg.position.y;
    pose(2) = tf::getYaw(msg.orientation);
    return pose;
  }

  inline geometry_msgs::Twist toTwistMsg(const Twist2d& twist)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = twist(0);
    msg.linear.y = twist(1);
    msg.angular.z = twist(2);
    return msg;
  }

  inline Twist2d fromTwistMsg(const geometry_msgs::Twist& msg)
  {
    Twist2d twist;
    twist(0) = msg.linear.x;
    twist(1) = msg.linear.y;
    twist(2) = msg.angular.z;
    return twist;
  }

  inline Twist2d changeReferenceFrame(const Pose2d& pose, const Twist2d& twist)
  {
    double angle = pose(2);
    Eigen::Matrix< double, 3, 3 > transform_matrix;
    using Eigen::operator<<;
    transform_matrix << cos(angle), -sin(angle),  0,
                 sin(angle),  cos(angle),  0,
                 0         ,  0         ,  1;
    return transform_matrix * twist;
  }

  class JacParams
  {
    public:
      JacParams() :
        lx(0.0), ly(0.0), drive_constant(0.0) {}
      JacParams(double lx, double ly, double drive_constant) :
        lx(lx), ly(ly), drive_constant(drive_constant) {}
      JacParams(const JacParams& other) :
        lx(other.lx), ly(other.ly), drive_constant(other.drive_constant) {}
      ~JacParams() {}
      double lx, ly, drive_constant;
  };

  inline OmniJac getJacobian(const JacParams& params)
  {
    using Eigen::operator<<;
    OmniJac jac;
    double a = 1.0 / (4.0 * params.drive_constant);
    double b = 1.0 / (4.0 * (params.lx + params.ly) * params.drive_constant);
    jac << a,  a,  a,  a,
          -a,  a,  a,  -a,
          -b, b, -b, b;
    return jac;
  }

  inline OmniJacInv getJacobianInverse(const JacParams& params)
  {
    using Eigen::operator<<;
    OmniJacInv jac;
    double a = params.drive_constant;
    double b = (params.lx + params.ly) * params.drive_constant;
    jac << a, -a, -b,
           a,  a,  b,
           a,  a, -b,
           a, -a,  b;
    return jac;
  }

  inline Twist2d omniFK(const JacParams& params, const OmniEncVel& delta_wheels)
  {
    using Eigen::operator*;
    return getJacobian(params) * delta_wheels;
  }

  inline OmniEncVel omniIK(const JacParams& params, const Twist2d& twist_2d)
  {
    using Eigen::operator*;
    return getJacobianInverse(params) * twist_2d;
  }

  inline Pose2d nextOdometry(const Pose2d& last_odom, const OmniEncVel& current_encoder_diff,
      const JacParams& params)
  {
    Twist2d twist_2d_in_base = omniFK(params, current_encoder_diff);
    Twist2d twist_2d_in_odom = changeReferenceFrame(last_odom, twist_2d_in_base);
    return twist_2d_in_odom + last_odom;
  }

  inline Twist2d limitTwist(const Twist2d& twist, const JacParams& params, double max_wheel_speed)
  {
    OmniEncVel wheel_speeds = omniIK(params, twist);
    double highest_wheel_speed = wheel_speeds.lpNorm<Eigen::Infinity>();
    if (std::abs(highest_wheel_speed) > 0.0 && // protect against division by zero
        std::abs(highest_wheel_speed) > std::abs(max_wheel_speed) )
      return std::abs(max_wheel_speed / highest_wheel_speed ) * twist;
    else
      return twist;
  }
}

#endif // OMNI_ETHERCAT_OMNILIB_HPP
