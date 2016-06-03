cmake_minimum_required(VERSION 2.8.3)
project(iai_kms_40_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES iai_kms_40_driver
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
)