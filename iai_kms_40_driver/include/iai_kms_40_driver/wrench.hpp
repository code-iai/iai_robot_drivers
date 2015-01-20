#ifndef IAI_KMS_40_DRIVER_WRENCH_H_
#define IAI_KMS_40_DRIVER_WRENCH_H_

#include <ostream>

namespace iai_kms_40_driver
{
  class Wrench
  {
    public:
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

#endif // IAI_KMS_40_DRIVER_WRENCH_H_
