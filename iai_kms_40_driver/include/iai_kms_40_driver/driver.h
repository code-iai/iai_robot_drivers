#ifndef IAI_KMS_40_DRIVER_DRIVER_H_
#define IAI_KMS_40_DRIVER_DRIVER_H_

#include <netdb.h>

#include <string>

namespace iai_kms_40_driver
{

  class KMS40Driver
  {
    public:
      KMS40Driver();
      ~KMS40Driver();

      bool init();

      void getTemperature();
      void getSingleWrench();


    private:
      struct addrinfo* host_info_list_;
      int socket_fd_;

      char readByte();
      std::string readLine();

      bool sendMessage(const std::string& msg);
  };

}
#endif // IAI_KMS_40_DRIVER_DRIVER_H_
