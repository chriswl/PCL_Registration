cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)

project(pcl_icp)

find_package(PCL 1.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable (pcl_icp main.cpp pcl_tools.cpp)
# target_link_libraries (pcl_icp ${PCL_LIBRARIES})

# add_executable (downsampler downsampler.cpp pcl_tools.cpp)
# target_link_libraries (downsampler ${PCL_LIBRARIES})
add_executable (combiner combiner.cpp pcl_tools.cpp)
target_link_libraries (combiner ${PCL_LIBRARIES} ${catkin_LIBRARIES})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)
