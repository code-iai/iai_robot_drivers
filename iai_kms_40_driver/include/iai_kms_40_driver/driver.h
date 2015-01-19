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

    private:
      struct addrinfo* host_info_list_;
      int socket_fd_;

      bool sendMessage(const std::string& msg);
      std::string readData(size_t bytesToRead);
  };

}
#endif // IAI_KMS_40_DRIVER_DRIVER_H_
