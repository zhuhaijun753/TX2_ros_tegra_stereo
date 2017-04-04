cmake_minimum_required(VERSION 2.8.3)
project(ros_tegra_stereo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge sensor_msgs image_transport)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
# To enable pkg-config usage
INCLUDE(FindPkgConfig)
# Find and add OpenCV to libraries
pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
# Find and add CUDA to libraries
pkg_check_modules(CUDArt REQUIRED cudart-8.0)
include_directories(${CUDArt_INCLUDE_DIRS})
link_directories(${CUDArt_LIBRARY_DIRS})
# Find and add VisionWorks to libraries
pkg_check_modules(VisionWorks REQUIRED visionworks)
include_directories(${VisionWorks_INCLUDE_DIRS})
link_directories(${VisionWorks_LIBRARY_DIRS})
# Add source folder
set(ALL_INCLUDE "src/")
include_directories(${ALL_INCLUDE})
# Include CATKIN packages
include_directories(include ${catkin_INCLUDE_DIRS})
# Compiler options
add_compile_options(-std=c++0x -DCUDA_API_PER_THREAD_DEFAULT_STREAM)

add_executable(sgbm_node
	        src/sgbm_disparity.cpp 
		src/Application.cpp
		src/ArgumentParser.cpp
		src/ConfigParser.cpp
		src/NVXIOFeatures.cpp
		src/OptionHandler.cpp
		src/stereo_matching.cpp
		src/Utility.cpp
		src/UtilityOVX.cpp
		)
target_link_libraries(sgbm_node
		${catkin_LIBRARIES} 
		${OpenCV_LIBRARIES}
		${CUDArt_LIBRARIES}
	        ${VisionWorks_LIBRARIES})

