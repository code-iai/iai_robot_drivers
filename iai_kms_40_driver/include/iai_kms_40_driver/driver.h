#ifndef IAI_KMS_40_DRIVER_DRIVER_H_
#define IAI_KMS_40_DRIVER_DRIVER_H_

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

      bool init(const std::string& ip, const std::string port,
          const timeval& read_timeout);

      bool start();
      void stop();

      Wrench currentWrench();

    private:
      SocketConnection socket_conn_;
      Wrench wrench_, wrench_buffer_;

      pthread_t thread_; 
      pthread_mutex_t mutex_; 
      bool exit_requested_;

      // actual function run be our thread
      void* run();
      // some interface function to feed run() to pthread_create
      static void* run_s(void *ptr) { return ((KMS40Driver *) ptr)->run(); }

      // various private aux functions
      bool requestStreamStart();
      bool requestStreamStop();
      void blockingReadWrench();
      void copyWrenchIntoBuffer();
  };

}
#endif // IAI_KMS_40_DRIVER_DRIVER_H_
