/*
 * Copyright (c) 2012, Ingo Kresse (ingo.kresse@in.tum.de)
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

#ifndef IAI_KMS_40_DRIVER_PTHREAD_SCOPED_LOCK_HPP_
#define IAI_KMS_40_DRIVER_PTHREAD_SCOPED_LOCK_HPP_

#include <pthread.h>

namespace iai_kms_40_driver
{
  class pthread_scoped_lock
  {
    public:
      pthread_scoped_lock(pthread_mutex_t *mutex) : mutex_(mutex) 
      { 
        pthread_mutex_lock(mutex_); 
      }

      ~pthread_scoped_lock() 
      { 
        unlock(); 
      }

      void unlock() 
      { 
        pthread_mutex_unlock(mutex_); 
      }

    private:
      pthread_mutex_t *mutex_;
  };
}

#endif // IAI_KMS_40_DRIVER_PTHREAD_SCOPED_LOCK_HPP_
