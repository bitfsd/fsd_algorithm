cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O2")
#set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        fsd_common_msgs
        fsd_tools
        )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif ()

find_package(IPOPT REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES hello
        CATKIN_DEPENDS roscpp std_msgs nav_msgs
        #   DEPENDS ipopt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
        /usr/include/eigen3
)

add_executable(${PROJECT_NAME}
        src/control.cpp
        src/control_handle.cpp
        src/main.cpp
        include/Utils/types.h
#        src/Track/track_base.cpp
#        src/Track/trackdrive_track.cpp
#        src/Track/line_track.cpp
#        src/Track/skidpad_track.cpp
        src/Solver/solver_base.cpp
        src/Solver/mpc_solver.cpp
        src/Solver/pure_pursuit_solver.cpp
        src/Utils/visual.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${IPOPT_LIBRARIES}
        ${OpenCV_LIBS}
        )

message(STATUS "IPOPT LIBRARIES:" ${IPOPT_LIBRARIES})