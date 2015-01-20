#include <iai_kms_40_driver/driver.h>
#include <iostream>

namespace iai_kms_40_driver
{
  KMS40Driver::KMS40Driver()
  {
  }

  KMS40Driver::~KMS40Driver()
  {
  }

  bool KMS40Driver::init()
  {
    return socket_conn_.open();
  }

  void KMS40Driver::getTemperature()
  {
    if( !socket_conn_.sendMessage("T()\n") )
      return;

    std::cout << socket_conn_.readLine() << std::endl;
  }

  void KMS40Driver::getSingleWrench()
  {
    if( !socket_conn_.sendMessage("F()\n") )
      return;

    std::cout << socket_conn_.readLine() << std::endl;
  }
}
