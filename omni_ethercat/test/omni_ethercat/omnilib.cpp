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

class OmnilibTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      params.lx = 0.39225;
      params.ly = 0.303495;
      params.drive_constant = 626594.7934;
    }

    virtual void TearDown(){}

    omni_ethercat::JacParams params;

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
