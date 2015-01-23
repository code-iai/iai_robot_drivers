#include <iai_kms_40_driver/kms_40_driver_node.hpp>
#include <iai_kms_40_driver/msg_conversions.hpp>
#include <iai_kms_40_driver/parser.hpp>

namespace iai_kms_40_driver
{
  KMS40DriverNode::KMS40DriverNode(const ros::NodeHandle& nh) : nh_(nh) 
  {
    pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
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
    std::string ip;

    if ( !nh_.getParam("ip", ip) )
    {
      ROS_ERROR("Could not find ROS parameter for IP");
      return false;
    }

    // specify a timeout of 1 second
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    read_timeout.tv_usec = 0;
    
    return driver_.start(ip, "1000", read_timeout);
  }
  
  void KMS40DriverNode::loop()
  {
    ros::Duration d(0.5);
    d.sleep();
  
    ros::Rate r(1);
    while(ros::ok())
    {
      pub_.publish(populateMsg(driver_.currentWrench(), msg_));
      ros::spinOnce();
      r.sleep();
    }
  }
} // namespace iai_kms_40_driver
