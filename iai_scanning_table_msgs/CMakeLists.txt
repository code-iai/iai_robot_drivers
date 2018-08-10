cmake_minimum_required(VERSION 2.8.3)
project(iai_scanning_table_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs std_msgs)


## Generate actions in the 'action' folder
  add_action_files(
    FILES
    scanning_table.action
  )

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs  # Or other packages containing msgs
  )

catkin_package()
