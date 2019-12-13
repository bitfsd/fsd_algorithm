cmake_minimum_required(VERSION 2.8.3)
project(template)

set(PROJECT_DEPS
  rospy
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)


catkin_python_setup()


install(DIRECTORY config launch msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS bin/${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
