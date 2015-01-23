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

#ifndef IAI_KMS_40_DRIVER_WRENCH_HPP_
#define IAI_KMS_40_DRIVER_WRENCH_HPP_

#include <ostream>

namespace iai_kms_40_driver
{
  class Wrench
  {
    public:
      Wrench() : 
          fx_(0.0), fy_(0.0), fz_(0.0), tx_(0.0), ty_(0.0), tz_(0.0), timestamp_(0)
      {}
      ~Wrench() {}
      double fx_, fy_, fz_, tx_, ty_, tz_;
      long timestamp_;
  };

  inline std::ostream& operator<< (std::ostream& stream, const Wrench& wrench)
  {
    stream << '{';
    stream << wrench.fx_ << " " << wrench.fy_ << " " << wrench.fz_ << " ";
    stream << wrench.tx_ << " " << wrench.ty_ << " " << wrench.tz_;
    stream << "} " << wrench.timestamp_;
    return stream;
  }
}

#endif // IAI_KMS_40_DRIVER_WRENCH_HPP_
