#ifndef IAI_KMS_40_DRIVER_MSG_CONVERSIONS_HPP_
#define IAI_KMS_40_DRIVER_MSG_CONVERSIONS_HPP_

#include <geometry_msgs/WrenchStamped.h>
#include <iai_kms_40_driver/wrench.hpp>

namespace iai_kms_40_driver
{
  geometry_msgs::WrenchStamped& populateMsg(const Wrench& wrench, 
      geometry_msgs::WrenchStamped& msg);
} // namespace iai_kms_40_driver
#endif // IAI_KMS_40_DRIVER_MSG_CONVERSIONS_HPP_
