#include <iai_kms_40_driver/msg_conversions.hpp>

namespace iai_kms_40_driver
{
  geometry_msgs::WrenchStamped& populateMsg(const Wrench& wrench,
      geometry_msgs::WrenchStamped& msg)
  {
    msg.wrench.force.x = wrench.fx_;
    msg.wrench.force.y = wrench.fy_;
    msg.wrench.force.z = wrench.fz_;
    msg.wrench.torque.x = wrench.tx_;
    msg.wrench.torque.y = wrench.ty_;
    msg.wrench.torque.z = wrench.tz_;

    return msg;
  }
} // namespace iai_kms_40_driver

