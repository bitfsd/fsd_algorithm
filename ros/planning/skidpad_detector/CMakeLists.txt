cmake_minimum_required(VERSION 2.8.3)
project(skidpad_detector)

set(CMAKE_BUILD_TYPE "Release")
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
  pcl_ros
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories(/usr/include/eigen3)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/skidpad_detector_handle.cpp
  src/skidpad_detector.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
