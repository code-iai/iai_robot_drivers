#ifndef IAI_KMS_40_DRIVER_NODE_HPP_
#define IAI_KMS_40_DRIVER_NODE_HPP_

#include <iai_kms_40_driver/driver.h>
#include <iai_kms_40_driver/parser.hpp>

namespace iai_kms_40_driver
{
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
}

#endif // IAI_KMS_40_DRIVER_NODE_HPP_
