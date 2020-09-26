cmake_minimum_required(VERSION 2.8.3)
project(lidar_cluster)
set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  fsd_common_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  fsd_common_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  )
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/lidar_cluster_handle.cpp
  src/lidar_cluster.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )
