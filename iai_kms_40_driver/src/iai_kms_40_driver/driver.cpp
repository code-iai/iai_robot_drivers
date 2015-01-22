#include <iai_kms_40_driver/driver.h>
#include <iai_kms_40_driver/parser.hpp>
#include <iai_kms_40_driver/pthread_scoped_lock.hpp>
#include <iostream>

namespace iai_kms_40_driver
{
  KMS40Driver::KMS40Driver() : exit_requested_( false )
  {
  }

  KMS40Driver::~KMS40Driver()
  {
    stop();
  }

  bool KMS40Driver::init(const std::string& ip, const std::string port,
      const timeval& read_timeout)
  {
    return socket_conn_.open(ip, port, read_timeout);
  }

  bool KMS40Driver::start()
  {
    if ( !requestStreamStart() )
    {
      std::cout << "Error during request to start streaming.\n";
      return false;
    } 

    std::cout << "Starting thread\n";

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

    if(pthread_create(&thread_, &tattr, &KMS40Driver::run_s, (void *) this) != 0) 
    {
      fprintf(stderr, "# ERROR: could not create realtime thread\n");                                   
      return false;                               
    }
  
    exit_requested_ = false;
    return true;
  }

  void KMS40Driver::stop()
  {
    pthread_mutex_lock(&mutex_);
    exit_requested_ = true;
    pthread_mutex_unlock(&mutex_);

    if ( !requestStreamStop() )
      std::cout << "Error during request to stop data streaming.\n";

    pthread_join(thread_, 0);
  }

  Wrench KMS40Driver::currentWrench()
  {
    pthread_scoped_lock lock(&mutex_);
    return wrench_buffer_;
  }

  bool KMS40Driver::requestStreamStart()
  {
    socket_conn_.sendMessage("L1()\n");
    
    return (socket_conn_.readChunk().compare("L1\n") == 0);
  }

  bool KMS40Driver::requestStreamStop()
  {
    socket_conn_.sendMessage("L0()\n");
    
    return (socket_conn_.readChunk().compare("L0\n") == 0);
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
      std::cout << "Error parsing wrench message!\nMessage: " << readData << "\n";
  }

  void KMS40Driver::copyWrenchIntoBuffer()
  {
    pthread_scoped_lock lock(&mutex_);
    wrench_buffer_ = wrench_;
  }
}
