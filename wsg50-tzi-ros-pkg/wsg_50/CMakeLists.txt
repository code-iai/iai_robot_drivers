cmake_minimum_required(VERSION 2.8.3)
project(wsg_50)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  wsg_50_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  std_srvs
#  wsg_50_common
#)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# WSG_50_TCP version

set(DRIVER_SOURCES 
  src/checksum.cpp
  src/cmd.c
  src/common.cpp
  src/functions.cpp
  src/interface.cpp
  src/main.cpp
  src/msg.c
  src/serial.c
  src/tcp.c
  src/udp.c)

add_executable(wsg_50_tcp src/main.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg_50_tcp ${catkin_LIBRARIES})


# WSG_50_CAN version

set(DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

#add_executable(wsg_50_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

add_definitions(-DOSNAME_LINUX)

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)

#set_target_properties(target PROPERTIES COMPILE_FLAGS wsg_50_can -g -Wall)
#target_link_libraries(wsg_50_can pcan)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    std_srvs
    std_msgs
    wsg_50_common
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
