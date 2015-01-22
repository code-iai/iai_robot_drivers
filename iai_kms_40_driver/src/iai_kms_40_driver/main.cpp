#include "ros/ros.h"

#include <iai_kms_40_driver/driver.h>
#include <iai_kms_40_driver/parser.hpp>

using namespace iai_kms_40_driver;

class KMS40DriverNode
{
  public:
    KMS40DriverNode(const ros::NodeHandle& nh) : nh_(nh) {}

    ~KMS40DriverNode() {}

    void run()
    {
      if(!startUp())
        return;

      loop();

      driver_.stop();
    }

  private:
    ros::NodeHandle nh_;
    KMS40Driver driver_;

    bool startUp()
    {
      // specify a timeout of 1 second
      struct timeval read_timeout;
      read_timeout.tv_sec = 1;
      read_timeout.tv_usec = 0;
      
      return driver_.init("192.168.100.175", "1000", read_timeout) && driver_.start();
    }

    void loop()
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kms40_node");
  ros::NodeHandle nh("~");
  KMS40DriverNode kms40Node(nh);
  kms40Node.run();
  return 0;
}
