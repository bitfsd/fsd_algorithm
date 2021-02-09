cmake_minimum_required(VERSION 2.8.3)
project(fsd_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigne_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

add_library(${PROJECT_NAME}
	src/cubic_spline.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
  )
