#ifndef IAI_KMS_40_DRIVER_KMS_40_DRIVER_NODE_HPP_
#define IAI_KMS_40_DRIVER_KMS_40_DRIVER_NODE_HPP_

#include <ros/ros.h>
#include <iai_kms_40_driver/driver.h>

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
      KMS40Driver driver_;
  
      bool startUp();
      void loop();
  };
} // namespace iai_kms_40_driver
#endif // IAI_KMS_40_DRIVER_KMS_40_DRIVER_NODE_HPP_
