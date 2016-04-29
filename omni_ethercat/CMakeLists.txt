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

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES vh_pixelmap
  CATKIN_DEPENDS roscpp std_msgs igh_eml geometry_msgs message_runtime tf diagnostic_updater iai_control_msgs soft_runstop
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(omni_ethercat src/omni_ethercat.cpp src/omnilib/omnilib.c src/omnilib/realtime.c)
target_link_libraries(omni_ethercat ${catkin_LIBRARIES})
add_dependencies(omni_ethercat ${catkin_EXPORTED_TARGETS})


##  #amaldo 20130726
##  #The following two variables keep the compilation rpath set on the binary
##  #so that the libraries are found when the binary is executed using sudo
##  #The sudo environment removes by default LD_LIBRARY_PATH and other non-system directories from the environment
##  # use, i.e. don't skip the full RPATH for the build tree
##  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
##  
##  # when building, don't use the install RPATH already
##  # (but later on when installing)
##  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
##  SET(CMAKE_INSTALL_RPATH "/opt/ros/hydro/lib/")
##  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
##  
##  #set the default path for built executables to the "bin" directory
##  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
