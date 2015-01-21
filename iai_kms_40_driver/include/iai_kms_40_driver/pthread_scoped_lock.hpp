#ifndef IAI_KMS_40_DRIVER_PTHREAD_SCOPED_LOCK_HPP_
#define IAI_KMS_40_DRIVER_PTHREAD_SCOPED_LOCK_HPP_

#include <pthread.h>

namespace iai_kms_40_driver
{
  //
  // A helper class to use pthread_locks like scoped locks from boost.
  //
  // This code was authored by Ingo Kresse, and copied here by Georg Bartels.
  //
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
