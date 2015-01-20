#ifndef IAI_KMS_40_DRIVER_SOCKET_CONNECTION_HPP_
#define IAI_KMS_40_DRIVER_SOCKET_CONNECTION_HPP_

#include <netdb.h>

#include <string>

namespace iai_kms_40_driver
{

  class SocketConnection
  {
    public:
      SocketConnection();
      ~SocketConnection();

      bool open();

      char readByte();
      std::string readLine();

      bool sendMessage(const std::string& msg);

    private:
      int socket_fd_;
  };
}
#endif // IAI_KMS_40_DRIVER_SOCKET_CONNECTION_HPP_
