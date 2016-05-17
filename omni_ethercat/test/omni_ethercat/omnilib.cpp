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

#include <gtest/gtest.h>
#include <omni_ethercat/omnilib.hpp>
#include <kdl/frames.hpp>

class OmnilibTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      params.lx = 0.39225;
      params.ly = 0.303495;
      params.drive_constant = 626594.7934;
      max_wheel_speed = 600000;
    }

    virtual void TearDown(){}

    omni_ethercat::JacParams params;
    double max_wheel_speed;

};

TEST_F(OmnilibTest, omniFKForewards)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << 0.1, 0.1, 0.1, 0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_GT(twist2d(0), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKBackwards)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << -0.1, -0.1, -0.1, -0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_LT(twist2d(0), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKLeft)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << -0.1, 0.1, 0.1, -0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_DOUBLE_EQ(twist2d(0), 0.0);
  EXPECT_GT(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKRight)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << 0.1, -0.1, -0.1, 0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_DOUBLE_EQ(twist2d(0), 0.0);
  EXPECT_LT(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKNorthWest)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << 0.0, 0.1, 0.1, 0.0;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_GT(twist2d(0), 0.0);
  EXPECT_GT(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKNorthEast)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << 0.1, 0.0, 0.0, 0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_GT(twist2d(0), 0.0);
  EXPECT_LT(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKSouthWest)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << -0.1, 0.0, 0.0, -0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_LT(twist2d(0), 0.0);
  EXPECT_GT(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKSouthEast)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << 0.0, -0.1, -0.1, 0.0;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_LT(twist2d(0), 0.0);
  EXPECT_LT(twist2d(1), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKRotPositive)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << -0.1, 0.1, -0.1, 0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_DOUBLE_EQ(twist2d(0), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(1), 0.0);
  EXPECT_GT(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniFKRotNegative)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  OmniEncVel omni_enc_vel;

  omni_enc_vel << 0.1, -0.1, 0.1, -0.1;
  Twist2d twist2d = omniFK(params, omni_enc_vel);
  EXPECT_DOUBLE_EQ(twist2d(0), 0.0);
  EXPECT_DOUBLE_EQ(twist2d(1), 0.0);
  EXPECT_LT(twist2d(2), 0.0);
}

TEST_F(OmnilibTest, omniIKForwards)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 1.0, 0.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_GT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(1));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(2));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
}

TEST_F(OmnilibTest, omniIKBackwards)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << -1.0, 0.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_LT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(1));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(2));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
}

TEST_F(OmnilibTest, omniIKLeft)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 0.0, 1.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_LT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
  EXPECT_GT(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(2));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), -omni_enc_vel(1));
}

TEST_F(OmnilibTest, omniIKRight)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 0.0, -1.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_GT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
  EXPECT_LT(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(2));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), -omni_enc_vel(1));
}

TEST_F(OmnilibTest, omniIKNorthWest)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 1.0, 1.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(3), 0.0);
  EXPECT_GT(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(2));
}

TEST_F(OmnilibTest, omniIKNorthEast)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 1.0, -1.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_GT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(2));
}

TEST_F(OmnilibTest, omniIKSouthEast)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << -1.0, -1.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
  EXPECT_LT(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(2));
}

TEST_F(OmnilibTest, omniIKSouthWest)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << -1.0, 1.0, 0.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_LT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(3));
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(2));
}

TEST_F(OmnilibTest, omniIKRotatePositive)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 0.0, 0.0, 1.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_LT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(2));
  EXPECT_GT(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(3));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), -omni_enc_vel(1));
}

TEST_F(OmnilibTest, omniIKRotateNegative)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist_2d;
  twist_2d << 0.0, 0.0, -1.0;

  OmniEncVel omni_enc_vel = omniIK(params, twist_2d);
  EXPECT_GT(omni_enc_vel(0), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), omni_enc_vel(2));
  EXPECT_LT(omni_enc_vel(1), 0.0);
  EXPECT_DOUBLE_EQ(omni_enc_vel(1), omni_enc_vel(3));
  EXPECT_DOUBLE_EQ(omni_enc_vel(0), -omni_enc_vel(1));
}

TEST_F(OmnilibTest, omniRoundtrip)
{
  Eigen::Matrix3d M = 
      omni_ethercat::getJacobian(params) * omni_ethercat::getJacobianInverse(params);
  EXPECT_EQ( M.cwiseEqual(Eigen::Matrix3d::Identity()).count(), M.rows()*M.cols() );
}

void expectPose2dEqual(const omni_ethercat::Pose2d& a, const omni_ethercat::Pose2d& b,
    double delta = 1.0e-15)
{
  EXPECT_DOUBLE_EQ(a(0), b(0));
  EXPECT_DOUBLE_EQ(a(1), b(1));

  // check that angle difference is zero
  double angle_diff = a(2) - b(2);
  angle_diff = std::fmod(angle_diff, 2.0*M_PI); // in range [0, 2PI[
  if(angle_diff > M_PI)
    angle_diff -= 2.0*M_PI; // in range [-PI, PI[
  angle_diff = std::abs(angle_diff); // in range [0, PI[
  EXPECT_LT(angle_diff, delta);
}

void expectTwist2dEqual(const omni_ethercat::Twist2d& a, const omni_ethercat::Twist2d& b)
{
  EXPECT_DOUBLE_EQ(a(0), b(0));
  EXPECT_DOUBLE_EQ(a(1), b(1));
  EXPECT_DOUBLE_EQ(a(2), b(2));
}

TEST_F(OmnilibTest, Pose2dMsgConversions)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Pose2d pose = Pose2d::Zero();
  expectPose2dEqual(pose, fromPoseMsg(toPoseMsg(pose)));
  pose << 1.1, -2.2, 3.3;
  expectPose2dEqual(pose, fromPoseMsg(toPoseMsg(pose)));
  pose << -0.1, -0.2, -0.3;
  expectPose2dEqual(pose, fromPoseMsg(toPoseMsg(pose)));
}

TEST_F(OmnilibTest, Twist2dMsgConversions)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  Twist2d twist = Twist2d::Zero();
  expectTwist2dEqual(twist, fromTwistMsg(toTwistMsg(twist)));
  twist << 0.1, 0.2, 0.3;
  expectTwist2dEqual(twist, fromTwistMsg(toTwistMsg(twist)));
  twist << -1.1, -2.2, -3.3;
  expectTwist2dEqual(twist, fromTwistMsg(toTwistMsg(twist)));
  twist << -1.1, 2.2, -13.3;
  expectTwist2dEqual(twist, fromTwistMsg(toTwistMsg(twist)));
}

omni_ethercat::Pose2d fromKDLFrame(const KDL::Frame& frame)
{
  omni_ethercat::Pose2d pose;
  pose(0) = frame.p.x();
  pose(1) = frame.p.y();
  KDL::Vector rot_axis(0.0, 0.0, 1.0);
  pose(2) = frame.M.GetRotAngle(rot_axis);
  return pose;
}

KDL::Frame toKDLFrame(const omni_ethercat::Pose2d& pose)
{
  return KDL::Frame(KDL::Rotation::RotZ(pose(2)), KDL::Vector(pose(0), pose(1), 0.0));
}

TEST_F(OmnilibTest, NextOdometry)
{
  using namespace omni_ethercat;
  KDL::Frame f1(KDL::Rotation::RotZ(M_PI/4.0), KDL::Vector(0.1, -0.2, 0.0));
  KDL::Frame f2(KDL::Rotation::RotZ(M_PI/8.0), KDL::Vector(0.15, -0.25, 0.0));
  EXPECT_TRUE(KDL::Equal(f1, toKDLFrame(fromKDLFrame(f1))));
  EXPECT_TRUE(KDL::Equal(f2, toKDLFrame(fromKDLFrame(f2))));

  // TEST CASE 1: one odometry step, IDENTITY -> IDENTITY
  expectPose2dEqual(Pose2d::Zero(), nextOdometry(Pose2d::Zero(), OmniEncVel::Zero(), params));

  // TEST CASE 2: one odometry step, IDENTITY -> F1
  expectPose2dEqual(fromKDLFrame(f1), 
      nextOdometry(Pose2d::Zero(), omniIK(params, fromKDLFrame(f1)), params));

  // TEST CASE 3: two odometry steps, IDENTIY -> F1 -> F1*F2
  expectPose2dEqual(fromKDLFrame(f1*f2), 
      nextOdometry(
          nextOdometry(Pose2d::Zero(), omniIK(params, fromKDLFrame(f1)), params),
          omniIK(params, fromKDLFrame(f2)), params));
}

TEST_F(OmnilibTest, LimitTwistWheelSpeed)
{
  using namespace omni_ethercat;
  using Eigen::operator<<;
  
  // TEST CASE 0: nothing for zero twist command
  Twist2d twist_cmd = Twist2d::Zero();
  Twist2d limited_twist_cmd = limitTwist(twist_cmd, params, max_wheel_speed);
  expectTwist2dEqual(limited_twist_cmd, twist_cmd);

  // TEST CASE 1: nothing for twist command with some zeros and low norm
  twist_cmd << 0.1, 0.0, 0.0;
  limited_twist_cmd = limitTwist(twist_cmd, params, max_wheel_speed);
  expectTwist2dEqual(limited_twist_cmd, twist_cmd);

  // TEST CASE 2: nothing for twist command with low norm
  twist_cmd << 0.1, 0.1, 0.1;
  limited_twist_cmd = limitTwist(twist_cmd, params, max_wheel_speed);
  expectTwist2dEqual(limited_twist_cmd, twist_cmd);

  // TEST CASE 3: limit twist command which has all wheels too fast
  twist_cmd << 0.0, 0.0, 2.0;
  limited_twist_cmd = limitTwist(twist_cmd, params, max_wheel_speed);
  OmniEncVel limited_wheel_speeds = omniIK(params, limited_twist_cmd);
  expectTwist2dEqual(limited_twist_cmd / limited_twist_cmd.norm(), twist_cmd / twist_cmd.norm());
  EXPECT_EQ((limited_wheel_speeds.array().abs() == OmniEncVel::Constant(std::abs(max_wheel_speed)).array()).count(), 4);

  // TEST CASE 4: limit twist command which has one wheel too fast
  twist_cmd << 0.7, -0.7, -0.7;
  limited_twist_cmd = limitTwist(twist_cmd, params, max_wheel_speed);
  limited_wheel_speeds = omniIK(params, limited_twist_cmd);
  expectTwist2dEqual(limited_twist_cmd / limited_twist_cmd.norm(), twist_cmd / twist_cmd.norm());
  EXPECT_EQ((limited_wheel_speeds.array().abs() < OmniEncVel::Constant(std::abs(max_wheel_speed)).array()).count(), 3);
  EXPECT_EQ((limited_wheel_speeds.array().abs() == OmniEncVel::Constant(std::abs(max_wheel_speed)).array()).count(), 1);
}
