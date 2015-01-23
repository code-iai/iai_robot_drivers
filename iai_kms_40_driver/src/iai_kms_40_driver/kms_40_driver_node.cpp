#include <iai_kms_40_driver/kms_40_driver_node.hpp>
#include <iai_kms_40_driver/msg_conversions.hpp>
#include <iai_kms_40_driver/parser.hpp>

namespace iai_kms_40_driver
{
  struct timeval convertTime(double seconds)
  {
    ros::Duration tmp(seconds);

    // specify a timeout of 1 second
    struct timeval result;
    result.tv_sec = tmp.sec;
    result.tv_usec = tmp.nsec / 1000;

    return result;
  }

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
    std::string ip, port;
    double timeout;

    if ( !nh_.getParam("ip", ip) )
    {
      ROS_ERROR("Could not find ROS parameter for IP");
      return false;
    }

    if ( !nh_.getParam("port", port) )
    {
      ROS_ERROR("Could not find ROS parameter for TCP port");
      return false;
    }

    if ( !nh_.getParam("tcp_timeout", timeout) )
    {
      ROS_ERROR("Could not find ROS parameter for TCP timeout");
      return false;
    }

    return driver_.start(ip, port, convertTime(timeout));
  }
  
  void KMS40DriverNode::loop()
  {
    int publish_rate;

    if ( !nh_.getParam("publish_rate", publish_rate) )
    {
      ROS_ERROR("Could not find ROS parameter for ROS publish rate");
      return;
    }

    if ( !(publish_rate > 0) )
    {
      ROS_ERROR("Given ROS publish rate was not greater 0");
      return;
    }

    ros::Rate r(publish_rate);
    while(ros::ok())
    {
      pub_.publish(populateMsg(driver_.currentWrench(), msg_));
      ros::spinOnce();
      r.sleep();
    }
  }
} // namespace iai_kms_40_driver
