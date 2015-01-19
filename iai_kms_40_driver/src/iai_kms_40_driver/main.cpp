#include "ros/ros.h"

#include <iai_kms_40_driver/driver.h>

using namespace iai_kms_40_driver;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kms40_node");
  ros::NodeHandle n("~");
  ROS_INFO("Running kms40 standalone node.\n");

  KMS40Driver my_driver;
  if(!my_driver.init())
//    ROS_ERROR("Error initializing kms40 driver.");
std::cout << "Error initializing kms40 driver.\n";

my_driver.getTemperature();
  return 0;
}
