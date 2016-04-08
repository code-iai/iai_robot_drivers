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

#include <iai_kms_40_driver/kms_40_driver.hpp>
#include <iai_kms_40_driver/parser.hpp>
#include <iai_kms_40_driver/pthread_scoped_lock.hpp>
#include <iostream>
#include <sstream>

namespace iai_kms_40_driver
{
  KMS40Driver::KMS40Driver() : exit_requested_( false ), running_( false )
  {
  }

  KMS40Driver::~KMS40Driver()
  {
    stop();
  }

  bool KMS40Driver::start(const std::string& ip, const std::string port,
      const timeval& read_timeout, unsigned int frame_divider)
  {
    if ( !socket_conn_.open(ip, port, read_timeout) )
    {
      std::cout << "Errr during opening of socket.\n";
      return false;
    }

    if ( !configureStream(frame_divider) )
    {
      std::cout << "Error during configuring of stream.\n";
      socket_conn_.shutdown();
      return false;
    }
    kmsServiceRequest("VL(1)\n", "VL=1\n");
    if ( !requestStreamStart() )
    {
      std::cout << "Error during request to start streaming.\n";
      socket_conn_.shutdown();
      return false;
    } 

    if ( !spinRealtimeThread() )
    {
      std::cout << "Error when spinning realtime thread.\n";
      requestStreamStop();
      socket_conn_.shutdown();
      return false;
    }
 
    exit_requested_ = false;
    running_ = true;
    return true;
  }

  void KMS40Driver::stop()
  {
    if(running_)
    {
      pthread_mutex_lock(&mutex_);
      exit_requested_ = true;
      pthread_mutex_unlock(&mutex_);
  
      if ( !requestStreamStop() )
        std::cout << "Error during request to stop data streaming.\n";
  
      struct timespec read_timeout;
      read_timeout.tv_sec = 1;
      read_timeout.tv_nsec = 0;
  
      pthread_join(thread_, 0);

      running_ = false;
    }

    socket_conn_.shutdown();
  }

  Wrench KMS40Driver::currentWrench()
  {
    pthread_scoped_lock lock(&mutex_);
    return wrench_buffer_;
  }

  bool KMS40Driver::spinRealtimeThread()
  {
    // setting up mutex
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);

    pthread_mutex_init(&mutex_,  &mattr);

    // setting up thread
    pthread_attr_t tattr;
    struct sched_param sparam;
    sparam.sched_priority = 12;
    pthread_attr_init(&tattr);
    pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
    pthread_attr_setschedparam(&tattr, &sparam);
    pthread_attr_setinheritsched (&tattr, PTHREAD_EXPLICIT_SCHED);
    int return_code = pthread_create(&thread_, &tattr, &KMS40Driver::run_s, (void *) this);
    if (return_code != 0) 
      std::cout << "return code: " << strerror(return_code) << std::endl;

    return (return_code == 0);
  }

  bool KMS40Driver::configureStream(unsigned int frame_divider)
  {
    std::ostringstream response, request;
    request << "LDIV(" << frame_divider << ")\n";
    response << "LDIV=" << frame_divider << "\n";

    return kmsServiceRequest(request.str(), response.str());
  } 

  bool KMS40Driver::requestStreamStart()
  {
    return kmsServiceRequest("L1()\n", "L1\n");
  }

  bool KMS40Driver::requestStreamStop()
  {
    return kmsServiceRequest("L0()\n", "L0\n");
  }

  void* KMS40Driver::run()
  {
    while( !exit_requested_ )
    {
      blockingReadWrench();
      copyWrenchIntoBuffer();
    }

    return 0;
  }

  void KMS40Driver::blockingReadWrench()
  {
    std::string readData = socket_conn_.readChunk();
    
    if( !parse_wrench(readData, wrench_) )
      std::cout << "Error parsing wrench message!\nMessage:\n" << readData << std::endl;
  }

  void KMS40Driver::copyWrenchIntoBuffer()
  {
    pthread_scoped_lock lock(&mutex_);
    wrench_buffer_ = wrench_;
  }

  bool KMS40Driver::kmsServiceRequest(const std::string& request, const std::string& response)
  {
    socket_conn_.sendMessage(request);
    return (socket_conn_.readChunk().compare(response) == 0);
  }
}
