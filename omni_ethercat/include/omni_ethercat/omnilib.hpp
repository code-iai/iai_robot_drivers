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
  typedef Eigen::Matrix< double, 3, 4 > Matrix34d;
  typedef Eigen::Matrix< double, 4, 3 > Matrix43d;
  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Vector4d Vector4d;

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

  Matrix34d getJacobian(double lx, double ly, double drive_constant)
  {
    using Eigen::operator<<;
    Matrix34d jac;
    double a = drive_constant * 4.0;
    double b = drive_constant * (lx + ly);
    jac << 1/a,  1/a,  1/a,  1/a,
          -1/a,  1/a,  1/a,  -1/a,
          -1/b, 1/b, -1/b, 1/b;
    return jac;
  }

  Matrix34d getJacobian(const JacParams& params)
  {
    return getJacobian(params.lx, params.ly, params.drive_constant);
  }

  Matrix43d getJacobianInverse(double lx, double ly, double drive_constant)
  {
    using Eigen::operator<<;
    Matrix43d jac;
    double a = drive_constant;
    double b = drive_constant/(lx + ly);
    jac << a, -a, -b,
           a,  a,  b,
           a,  a, -b,
           a, -a,  b;
    return jac;
  }

  Matrix43d getJacobianInverse(const JacParams& params)
  {
    return getJacobianInverse(params.lx, params.ly, params.drive_constant);
  }
 
  Vector3d omniFK(double lx, double ly, double drive_constant, const Vector4d& delta_wheels)
  {
    using Eigen::operator*;
    return getJacobian(lx, ly, drive_constant) * delta_wheels;
  }

  Vector3d omniFK(const JacParams& params, const Vector4d& delta_wheels)
  {
    return omniFK(params.lx, params.ly, params.drive_constant, delta_wheels);
  }

  Vector4d omniIK(double lx, double ly, double drive_constant, const Vector3d& twist_2d)
  {
    using Eigen::operator*;
    return getJacobianInverse(lx, ly, drive_constant) * twist_2d;
  }

  Vector4d omniIK(const JacParams& params, const Vector3d& twist_2d)
  {
    return omniIK(params.lx, params.ly, params.drive_constant, twist_2d);
  }
}

#endif // OMNI_ETHERCAT_OMNILIB_HPP
