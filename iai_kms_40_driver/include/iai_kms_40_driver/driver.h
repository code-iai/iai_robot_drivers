#ifndef IAI_KMS_40_DRIVER_DRIVER_H_
#define IAI_KMS_40_DRIVER_DRIVER_H_

#include <iai_kms_40_driver/socket_connection.hpp>
#include <iai_kms_40_driver/wrench.hpp>

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
      Wrench last_wrench_;
  };

}
#endif // IAI_KMS_40_DRIVER_DRIVER_H_
