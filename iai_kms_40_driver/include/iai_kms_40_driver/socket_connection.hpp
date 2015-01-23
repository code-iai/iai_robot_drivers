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

      bool open(const std::string& ip, const std::string port, 
          const timeval& read_timeout);
      
      void shutdown();

      bool ready() const;

      bool sendMessage(const std::string& msg);
      std::string readChunk();

    private:
      int socket_fd_;
      timeval read_timeout_;

      static const size_t buffer_size_ = 1024;
  };
}
#endif // IAI_KMS_40_DRIVER_SOCKET_CONNECTION_HPP_
