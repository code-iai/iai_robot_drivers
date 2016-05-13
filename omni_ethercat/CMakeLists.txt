cmake_minimum_required(VERSION 2.8.3)
project(omni_ethercat)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  igh_eml
  geometry_msgs
  message_runtime
  tf
  diagnostic_updater
  iai_control_msgs
  soft_runstop
)

find_package(Eigen3 REQUIRED)

catkin_package(

  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    igh_eml 
    geometry_msgs 
    message_runtime 
    tf 
    diagnostic_updater 
    iai_control_msgs 
    soft_runstop
  DEPENDS
    Eigen
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIR})

add_executable(omni_ethercat src/omni_ethercat.cpp src/omnilib/omnilib.c src/omnilib/realtime.c)
target_link_libraries(omni_ethercat ${catkin_LIBRARIES})
# NOTE: The following line is needed to halt our compilation until the CMake target
#       upstream_igh_eml which is declared in package igh_eml has built. It would
#       be nice to get this name through some variable. But I do not know how to do this.
add_dependencies(omni_ethercat upstream_igh_eml)

catkin_add_gtest(${PROJECT_NAME}-test test/${PROJECT_NAME}/omnilib.cpp test/main.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
  add_dependencies(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
