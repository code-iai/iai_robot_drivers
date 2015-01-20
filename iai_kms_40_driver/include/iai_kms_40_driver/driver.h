#ifndef IAI_KMS_40_DRIVER_DRIVER_H_
#define IAI_KMS_40_DRIVER_DRIVER_H_

//#include <netdb.h>

#include <string>
#include <iai_kms_40_driver/socket_connection.hpp>

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
      SocketConnection socket_conn_;;
  };

}
#endif // IAI_KMS_40_DRIVER_DRIVER_H_