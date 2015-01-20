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
//    ROS_ERROR("Error initializing kms40 driver.");
std::cout << "Error initializing kms40 driver.\n";

my_driver.getTemperature();
my_driver.getSingleWrench();
my_driver.getTemperature();
my_driver.getSingleWrench();
my_driver.getTemperature();
my_driver.getSingleWrench();
my_driver.getTemperature();
my_driver.getSingleWrench();

std::cout << "\n\n\n\n";

my_driver.getSingleWrench();
my_driver.getSingleWrench();
my_driver.getSingleWrench();
my_driver.getSingleWrench();
my_driver.getSingleWrench();
my_driver.getSingleWrench();
my_driver.getSingleWrench();
my_driver.getSingleWrench();


std::cout << "\n\n\n\n";

my_driver.getTemperature();
my_driver.getSingleWrench();
my_driver.getTemperature();
my_driver.getSingleWrench();
my_driver.getTemperature();
my_driver.getSingleWrench();
my_driver.getTemperature();
my_driver.getSingleWrench();
 
std::string str = "F={2.792,4.377,1.630,0.045,0.061,-0.000},166107629\n";
Wrench wrench;
if (parse_wrench(str, wrench))
        {
            std::cout << "-------------------------\n";
            std::cout << "Parsing succeeded\n";
            std::cout << str << " Parses OK: " << wrench << std::endl;
        }
        else
        {
            std::cout << "-------------------------\n";
            std::cout << "Parsing failed\n";
            std::cout << "-------------------------\n";
        }
 return 0;
}
