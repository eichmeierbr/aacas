cmake_minimum_required(VERSION 2.8.3)
project(aacas_motion)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  tf
  roscpp
  robot_state_publisher
  message_generation
#  aacas_detection
#  dji_m600_sim
  lidar_process
  dji_sdk
)



