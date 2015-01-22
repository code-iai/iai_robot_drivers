#include <iai_kms_40_driver/socket_connection.hpp>
#include <iostream>
#include <string.h>
#include <assert.h>

namespace iai_kms_40_driver
{
  SocketConnection::SocketConnection() :
      socket_fd_( -1 ), read_timeout_( )
  {
  }


  SocketConnection::~SocketConnection()
  {

  }

  bool SocketConnection::open(const std::string& ip, const std::string port,
     const timeval& read_timeout)
  {
    // copy over timeout
    read_timeout_ = read_timeout;

    // getting host info
    struct addrinfo host_info;
    struct addrinfo* host_info_list;
    memset(&host_info, 0, sizeof(host_info));
    host_info.ai_family = AF_UNSPEC;
    host_info.ai_socktype = SOCK_STREAM;
    int status = getaddrinfo(ip.c_str(), port.c_str(), &host_info, &host_info_list);
 
    if(status != 0)
    {
      std::cout << "Error getting host info.\n";
     
      return false;
    }

    // creating the socket
    socket_fd_ = socket(host_info_list->ai_family, host_info_list->ai_socktype,
        host_info_list->ai_protocol);
    if (socket_fd_ == -1)
    {  
      std::cout << "Error creating the socket.\n";
      freeaddrinfo(host_info_list);

      return false; 
    }

    // connecting the socket
    status = connect(socket_fd_, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)
    {
      std::cout << "Error connecting the socket.\n";
      freeaddrinfo(host_info_list);

      return false;
    }

    // free host info list
    freeaddrinfo(host_info_list);

    return true;
  }

  bool SocketConnection::ready() const
  {
    return (socket_fd_ != -1);
  }

  std::string SocketConnection::readChunk()
  {
    assert(ready());

    char in_buffer[buffer_size_];

    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (char *)&read_timeout_,
        sizeof(struct timeval));

    size_t bytes_received = recv(socket_fd_, &in_buffer, buffer_size_, 0);
    // If no data arrives, the program will just wait here until it times out

    if (bytes_received == 0)
      std::cout << "Error during reading: host shut down." << std::endl ;
    if (bytes_received == -1) 
      std::cout << "Error during reading: receive error!" << std::endl ;

    return std::string(in_buffer, bytes_received);
  }

  bool SocketConnection::sendMessage(const std::string& msg)
  {
    assert(ready());

    size_t bytes_sent = send(socket_fd_, msg.c_str(), msg.length(), 0);

    if ( (bytes_sent == -1) || (bytes_sent != msg.length()) )
    {
      std::cout << "An error occured during msg sending.\n";
      return false;
    }

    return true;
  }
} // namespace iai_kms_40_driver
