#include "ros/ros.h"

#include <iai_kms_40_driver/driver.h>
#include <iai_kms_40_driver/parser.hpp>

using namespace iai_kms_40_driver;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kms40_node");
  ros::NodeHandle n("~");
  ROS_INFO("Running kms40 standalone node.\n");

  KMS40Driver my_driver;
  if(!my_driver.init("192.168.100.175", "1000"))
    std::cout << "Error initializing kms40 driver.\n";

  std::cout << "Starting the node\n";
  my_driver.start();
  ros::Rate r(1);
  while(ros::ok())
  {
    std::cout << my_driver.currentWrench() << std::endl;
    ros::spinOnce();
    r.sleep();
  }
  std::cout << "Requesting to stop the node\n";
  my_driver.stop();

 return 0;
}
