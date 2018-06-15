cmake_minimum_required(VERSION 2.8.3)
project(omni_ethercat)


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        igh_eml
        geometry_msgs
        message_runtime
        diagnostic_updater
        iai_control_msgs
        soft_runstop
        sensor_msgs
        reflexxes_type2
        )

find_package(Eigen3 REQUIRED)

catkin_package(

        CATKIN_DEPENDS
        roscpp
        std_msgs
        igh_eml
        geometry_msgs
        message_runtime
        diagnostic_updater
        iai_control_msgs
        soft_runstop
        sensor_msgs
        reflexxes_type2
        DEPENDS
        Eigen
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIR})


#Since we will use setcap to give access to realtime priority without having root
# the LD_LIBRARY_PATH will be ignored when running the executable
# to be able to still find the libraries, record the current LD_LIBRARY_PATH in
# the executables

SET(CMAKE_SKIP_BUILD_RPATH FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH $ENV{LD_LIBRARY_PATH})
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


#add_executable(omni_ethercat src/omni_ethercat.cpp src/omnilib/omnilib.c src/omnilib/realtime.c)
add_executable(omni_ethercat2 src/omni_ethercat2.cpp src/ecat_iface.cpp src/interpolator.cpp)

#target_link_libraries(omni_ethercat ${catkin_LIBRARIES})
target_link_libraries(omni_ethercat2 ${catkin_LIBRARIES})

# NOTE: The following line is needed to halt our compilation until the CMake target
#       upstream_igh_eml which is declared in package igh_eml has built. It would
#       be nice to get this name through some variable. But I do not know how to do this.
#add_dependencies(omni_ethercat upstream_igh_eml)
add_dependencies(omni_ethercat2 upstream_igh_eml reflexxes_type2)


catkin_add_gtest(${PROJECT_NAME}-test test/${PROJECT_NAME}/omnilib.cpp test/main.cpp)

if (TARGET ${PROJECT_NAME}-test)
    find_package(orocos_kdl REQUIRED)
    include_directories(${orocos_kdl_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
    add_dependencies(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
endif ()

#MESSAGE(WARNING "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

