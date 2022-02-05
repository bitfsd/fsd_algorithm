cmake_minimum_required(VERSION 2.8.3)
project(path_generator)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O2")

set(PROJECT_DEPS
        roscpp
        std_msgs
        )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
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
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp std_msgs nav_msgs
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        /usr/include/eigen3
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
        src/path_generator_handle.cpp
        src/path_generator.cpp
        src/main.cpp
        src/Track/line_track.cpp
        src/Track/skidpad_track.cpp
        src/Track/trackdrive_track.cpp
        src/Track/track_base.cpp
        src/Utils/visual.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )