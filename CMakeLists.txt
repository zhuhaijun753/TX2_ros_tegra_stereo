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

set(NVX_INCLUDE "nvxio/include/")
include_directories(${NVX_INCLUDE})

set(NVX_SRC "nvxio/src/")
include_directories(${NVX_SRC})

set(OBJ_FLAGS "nvxio/obj/release")


set(OVX_STATIC_LIBS "libs/aarch64/linux/release")

include_directories(include ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++0x -DCUDA_API_PER_THREAD_DEFAULT_STREAM)

add_executable(subscriber
	       src/my_subscriber.cpp 
	       ${NVX_SRC}/NVX/Application.cpp
	       ${NVX_SRC}/NVX/ConfigParser.cpp
	       ${NVX_SRC}/NVX/SyncTimer.cpp
	       #${NVX_SRC}/OVX/UtilityOVX.cpp
	       #${NVX_SRC}/OVX/Render/RenderOVX.cpp
	       #${NVX_SRC}/OVX/FrameSource/FrameSourceOVX.cpp	
               ${NVX_INCLUDE}/NVX/Application.hpp 
	       ${NVX_INCLUDE}/NVX/ConfigParser.hpp
               ${NVX_INCLUDE}/NVX/SyncTimer.hpp 
	       #${NVX_INCLUDE}/OVX/UtilityOVX.hpp
               #${NVX_INCLUDE}/OVX/RenderOVX.hpp 
	       #${NVX_INCLUDE}/OVX/FrameSourceOVX.hpp
		)
target_link_libraries(subscriber 
		${catkin_LIBRARIES} 
		${OpenCV_LIBRARIES}
		${CUDArt_LIBRARIES}
	        ${VisionWorks_LIBRARIES}
		${OVX_STATIC_LIBS}/libnvx.a
		${OVX_STATIC_LIBS}/libovx.a)

