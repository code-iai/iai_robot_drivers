#include "ros/ros.h"

#include <iai_kms_40_driver/driver.h>
#include <iai_kms_40_driver/parser.hpp>

using namespace iai_kms_40_driver;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kms40_node");
  ros::NodeHandle n("~");

  // specify a timeout of 1 second
  struct timeval read_timeout;
  read_timeout.tv_sec = 1;
  read_timeout.tv_usec = 0;

  KMS40Driver my_driver;
  if(!my_driver.init("192.168.100.175", "1000", read_timeout))
  {
    ROS_ERROR("Error initializing kms40 driver.");
    return 0;
  }

  if(!my_driver.start())
  {
    ROS_ERROR("Error starting kms40 driver.");
    return 0;
  }

  ros::Duration d(0.5);
  d.sleep();

  ros::Rate r(1);
  while(ros::ok())
  {
    std::cout << my_driver.currentWrench() << std::endl;
    ros::spinOnce();
    r.sleep();
  }
  my_driver.stop();

  return 0;
}
