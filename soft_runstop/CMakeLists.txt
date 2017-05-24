cmake_minimum_required(VERSION 2.8.3)
project(soft_runstop)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)


include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs)

add_executable(soft_runstop src/soft_runstop.cc)
target_link_libraries(soft_runstop ${catkin_LIBRARIES})
add_dependencies(soft_runstop ${catkin_EXPORTED_TARGETS})



#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(TARGETS soft_runstop
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

