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

namespace omni_ethercat
{
  typedef Eigen::Matrix< double, 3, 4 > OmniJac;
  typedef Eigen::Matrix< double, 4, 3 > OmniJacInv;
  typedef Eigen::Vector3d Twist2d;
  typedef Eigen::Vector4d OmniEncVel;

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

  OmniJac getJacobian(double lx, double ly, double drive_constant)
  {
    using Eigen::operator<<;
    OmniJac jac;
    double a = drive_constant * 4.0;
    double b = drive_constant * (lx + ly);
    jac << 1/a,  1/a,  1/a,  1/a,
          -1/a,  1/a,  1/a,  -1/a,
          -1/b, 1/b, -1/b, 1/b;
    return jac;
  }

  OmniJac getJacobian(const JacParams& params)
  {
    return getJacobian(params.lx, params.ly, params.drive_constant);
  }

  OmniJacInv getJacobianInverse(double lx, double ly, double drive_constant)
  {
    using Eigen::operator<<;
    OmniJacInv jac;
    double a = drive_constant;
    double b = drive_constant/(lx + ly);
    jac << a, -a, -b,
           a,  a,  b,
           a,  a, -b,
           a, -a,  b;
    return jac;
  }

  OmniJacInv getJacobianInverse(const JacParams& params)
  {
    return getJacobianInverse(params.lx, params.ly, params.drive_constant);
  }
 
  Twist2d omniFK(double lx, double ly, double drive_constant, const OmniEncVel& delta_wheels)
  {
    using Eigen::operator*;
    return getJacobian(lx, ly, drive_constant) * delta_wheels;
  }

  Twist2d omniFK(const JacParams& params, const OmniEncVel& delta_wheels)
  {
    return omniFK(params.lx, params.ly, params.drive_constant, delta_wheels);
  }

  OmniEncVel omniIK(double lx, double ly, double drive_constant, const Twist2d& twist_2d)
  {
    using Eigen::operator*;
    return getJacobianInverse(lx, ly, drive_constant) * twist_2d;
  }

  OmniEncVel omniIK(const JacParams& params, const Twist2d& twist_2d)
  {
    return omniIK(params.lx, params.ly, params.drive_constant, twist_2d);
  }
}

#endif // OMNI_ETHERCAT_OMNILIB_HPP
