cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_common)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Cmd.msg
  Status.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Conf.srv
  Incr.srv
  Move.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime
    std_msgs
)

