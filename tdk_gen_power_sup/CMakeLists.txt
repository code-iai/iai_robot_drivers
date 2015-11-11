cmake_minimum_required(VERSION 2.8.3)
project(tdk_gen_power_sup)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  genesys_status.msg)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
