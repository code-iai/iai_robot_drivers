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

#ifndef IAI_KMS_40_DRIVER_KMS_40_DRIVER_HPP_
#define IAI_KMS_40_DRIVER_KMS_40_DRIVER_HPP_

#include <pthread.h>

#include <iai_kms_40_driver/socket_connection.hpp>
#include <iai_kms_40_driver/wrench.hpp>

namespace iai_kms_40_driver
{
  class KMS40Driver
  {
    public:
      KMS40Driver();
      ~KMS40Driver();

      bool start(const std::string& ip, const std::string port,
          const timeval& read_timeout, unsigned int frame_divider);

      void stop();

      Wrench currentWrench();
      bool publicKmsServiceRequest(const std::string& request, const std::string& response);

    private:
      SocketConnection socket_conn_;
      Wrench wrench_, wrench_buffer_;

      pthread_t thread_; 
      pthread_mutex_t mutex_; 
      bool exit_requested_, running_, paused_;

      // actual function run be our thread
      void* run();
      // some interface function to feed run() to pthread_create
      static void* run_s(void *ptr) { return ((KMS40Driver *) ptr)->run(); }

      // various private aux functions
      bool kmsServiceRequest(const std::string& request, const std::string& response);
      bool spinRealtimeThread();
      bool configureStream(unsigned int frame_divider);
      bool requestStreamStart();
      bool requestStreamStop();
      void blockingReadWrench();
      void copyWrenchIntoBuffer();
  };
}
#endif // IAI_KMS_40_DRIVER_KMS_40_DRIVER_HPP_
