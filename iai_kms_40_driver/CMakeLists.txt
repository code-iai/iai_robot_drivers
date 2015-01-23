cmake_minimum_required(VERSION 2.8.3)
project(iai_kms_40_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/socket_connection.cpp
  src/${PROJECT_NAME}/kms_40_driver.cpp
  src/${PROJECT_NAME}/msg_conversions.cpp
  src/${PROJECT_NAME}/kms_40_driver_node.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_executable(kms40_node 
  src/${PROJECT_NAME}/main.cpp)
target_link_libraries(kms40_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})
