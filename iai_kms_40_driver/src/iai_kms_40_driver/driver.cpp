#include <iai_kms_40_driver/driver.h>
#include <iostream>
#include <string.h>
#include <assert.h>

namespace iai_kms_40_driver
{
  KMS40Driver::KMS40Driver() : 
      host_info_list_( NULL ), socket_fd_( -1 )
  {
  }

  KMS40Driver::~KMS40Driver()
  {
    if(host_info_list_)
      freeaddrinfo(host_info_list_);

    if(socket_fd_ != -1)
      close(socket_fd_);
  }

  bool KMS40Driver::init()
  {
    // getting host info
    struct addrinfo host_info;
    memset(&host_info, 0, sizeof(host_info));
    host_info.ai_family = AF_UNSPEC;
    host_info.ai_socktype = SOCK_STREAM;
    int status = getaddrinfo("192.168.1.30", "1000", &host_info, &host_info_list_);
 
    if(status != 0)
    {
      std::cout << "Error getting host info.\n";
      return false;
    }

    // creating the socket
    assert(host_info_list_);
    socket_fd_ = socket(host_info_list_->ai_family, host_info_list_->ai_socktype, 
host_info_list_->ai_protocol);
    if (socket_fd_ == -1)
    {  
      std::cout << "Error creating the socket.\n";
      return false; 
    }

    // connecting the socket
    assert(host_info_list_);
    status = connect(socket_fd_, host_info_list_->ai_addr, host_info_list_->ai_addrlen);
    if (status == -1)
    {
      std::cout << "Error connecting the socket.\n";
      return false;
    }

    return true;
  }

  bool KMS40Driver::sendMessage(const std::string& msg)
  {
    size_t bytes_sent = send(socket_fd_, msg.c_str(), msg.length(), 0);

    if (bytes_sent == -1)
    {
      std::cout << "An error occured during msg sending.\n";
      return false;
    }

    return true;
  }

  void KMS40Driver::getTemperature()
  {
    if( !sendMessage("T()\n") )
      return;

    std::cout << readData(8) << std::endl;
  }

  std::string KMS40Driver::readData(size_t bytesToRead)
  {
    char in_buffer[bytesToRead];
    size_t bytes_received = recv(socket_fd_, in_buffer, bytesToRead, 0);
    // If no data arrives, the program will just wait here until some data arrives.

    if (bytes_received == 0)
      std::cout << "Error during reading: host shut down." << std::endl ;
    if (bytes_received == -1) 
      std::cout << "Error during reading: receive error!" << std::endl ;

    return std::string(in_buffer);
  }
}
