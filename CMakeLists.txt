cmake_minimum_required(VERSION 2.8.3)
project(kinect_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  )

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinect_calibration
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
