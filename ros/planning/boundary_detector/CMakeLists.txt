cmake_minimum_required(VERSION 2.8.3)
project(boundary_detector)

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
  tf
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

find_package(Eigen3 QUIET)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/boundaryDetector_handle.cpp
  src/boundaryDetector.cpp
  src/visual_path.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )
