cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)

project(pcl_registration)

include_directories(include
    ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARIES})

find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_icp src/main.cpp
    src/pcl_tools.cpp
    src/allparams.cpp
    src/cl_options.cpp)
target_link_libraries (pcl_icp ${PCL_LIBRARIES}
    boost_program_options
    )

