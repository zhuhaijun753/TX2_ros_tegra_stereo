cmake_minimum_required(VERSION 2.8.3)
project(ros_tegra_stereo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge sensor_msgs image_transport)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

INCLUDE(FindPkgConfig)

pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(subscriber src/my_subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(subscriber ros-tegra-stereo_cpp)
