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

pkg_check_modules(CUDArt REQUIRED cudart-8.0)
include_directories(${CUDArt_INCLUDE_DIRS})
link_directories(${CUDArt_LIBRARY_DIRS})

pkg_check_modules(VisionWorks REQUIRED visionworks)
include_directories(${VisionWorks_INCLUDE_DIRS})
link_directories(${VisionWorks_LIBRARY_DIRS})

set(ALL_INCLUDE "src/")
include_directories(${ALL_INCLUDE})

include_directories(include ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++0x -DCUDA_API_PER_THREAD_DEFAULT_STREAM)

add_executable(subscriber
	        src/my_subscriber.cpp 
		src/Application.cpp
		src/ArgumentParser.cpp
		src/color_disparity_graph.cpp
		src/ConfigParser.cpp
		src/NVXIOFeatures.cpp
		src/OptionHandler.cpp
		src/ProfilerRange.cpp
		src/stereo_matching.cpp
		src/Utility.cpp
		src/UtilityOVX.cpp
		)
target_link_libraries(subscriber 
		${catkin_LIBRARIES} 
		${OpenCV_LIBRARIES}
		${CUDArt_LIBRARIES}
	        ${VisionWorks_LIBRARIES})

