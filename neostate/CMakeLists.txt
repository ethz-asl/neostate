cmake_minimum_required(VERSION 2.8.3)
project(neostate)

find_package(catkin REQUIRED COMPONENTS 
rosserial_arduino
rosserial_python
rosserial_client
rospy
neostate_msgs
)

catkin_package(
  CATKIN_DEPENDS
    neostate_msgs
    rospy
    )

catkin_python_setup()

include_directories(include ${catkin_INCLUDE_DIRS})
