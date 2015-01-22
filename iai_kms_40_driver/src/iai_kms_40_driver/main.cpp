#include "ros/ros.h"
#include <iai_kms_40_driver/node.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kms40_node");
  ros::NodeHandle nh("~");
  iai_kms_40_driver::KMS40DriverNode kms40Node(nh);
  kms40Node.run();
  return 0;
}
