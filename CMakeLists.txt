cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)

project(pcl_registration)

find_package(PCL 1.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_icp src/main.cpp src/pcl_tools.cpp)
target_link_libraries (pcl_icp ${PCL_LIBRARIES})

add_executable (downsampler src/downsampler.cpp src/pcl_tools.cpp)
target_link_libraries (downsampler ${PCL_LIBRARIES})

