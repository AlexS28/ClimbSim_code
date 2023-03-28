cmake_minimum_required(VERSION 2.8.3)
project(joystick_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#catkin_python_setup()
catkin_install_python(PROGRAMS src/joystick.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
