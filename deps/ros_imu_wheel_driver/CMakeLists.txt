cmake_minimum_required(VERSION 3.5.0)
project(ros_imu_wheel_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system date_time)

find_package(Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_imu_cam_sync
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

set(SRCS src/main.cpp src/async-serial.cpp src/buffered-async-serial.cpp)
add_executable(${PROJECT_NAME}_node ${SRCS})
target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${glog_catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${gflags_catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${veye_catkin_LIBRARIES})

##target_link_libraries(${PROJECT_NAME}_node /usr/lib/libdmipicam.so)

##########
# GTESTS #
##########
catkin_add_gtest(test_utility
  tests/test-utility.cpp
)
target_link_libraries(test_utility ${glog_catkin_LIBRARIES})
target_link_libraries(test_utility ${gflags_catkin_LIBRARIES})
target_link_libraries(test_utility ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(test_utility ${catkin_LIBRARIES})

#############
## Install ##
#############

cs_install()
cs_export()