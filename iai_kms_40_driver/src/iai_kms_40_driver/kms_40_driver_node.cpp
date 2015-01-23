#include <iai_kms_40_driver/kms_40_driver_node.hpp>
#include <iai_kms_40_driver/parser.hpp>

namespace iai_kms_40_driver
{
  KMS40DriverNode::KMS40DriverNode(const ros::NodeHandle& nh) : nh_(nh) 
  {
  }
  
  KMS40DriverNode::~KMS40DriverNode()
  { 
  }
  
  void KMS40DriverNode::run()
  {
    if(!startUp())
      return;
  
    loop();
  
    driver_.stop();
  }
  
  bool KMS40DriverNode::startUp()
  {
    // specify a timeout of 1 second
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    read_timeout.tv_usec = 0;
    
    return driver_.start("192.168.100.175", "1000", read_timeout);
  }
  
  void KMS40DriverNode::loop()
  {
    ros::Duration d(0.5);
    d.sleep();
  
    ros::Rate r(1);
    while(ros::ok())
    {
      std::cout << driver_.currentWrench() << std::endl;
      ros::spinOnce();
      r.sleep();
    }
  }
} // namespace iai_kms_40_driver
