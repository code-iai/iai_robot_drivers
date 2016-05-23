/*
 * Copyright (c) 2015, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iai_kms_40_driver/socket_connection.hpp>
#include <iostream>
#include <string.h>
#include <assert.h>
#include <unistd.h>

namespace iai_kms_40_driver
{
  SocketConnection::SocketConnection() :
      socket_fd_( -1 ), read_timeout_( )
  {
  }


  SocketConnection::~SocketConnection()
  {
    shutdown();
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
    std::cout << host_info_list->ai_protocol << std::endl;
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

  void SocketConnection::shutdown()
  {
    if(ready())
      close(socket_fd_);
    socket_fd_ = -1;
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
    {
      std::cout << "Error during reading: host shut down." << std::endl ;
      return "";
    }
    if (bytes_received == -1) 
    {
      std::cout << "Error during reading: receive error!" << std::endl ;
      return "";
    }
    std::string response = std::string(in_buffer, bytes_received);
    std::cout << "reading: " << response << "\n";
    return response;
  }

  bool SocketConnection::sendMessage(const std::string& msg)
  {
    assert(ready());
    std::cout << "sending: " << msg << "\n";
    size_t bytes_sent = send(socket_fd_, msg.c_str(), msg.length(), 0);

    if ( (bytes_sent == -1) || (bytes_sent != msg.length()) )
    {
      std::cout << "Error sending a message.\n";
      return false;
    }

    return true;
  }
} // namespace iai_kms_40_driver
