cmake_minimum_required(VERSION 2.8.3)
project(aacas_detection)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  lidar_process
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  roscpp
  robot_state_publisher
  message_generation
)


add_service_files(
  DIRECTORY srv
  FILES
#  DroneTaskControl.srv
  QueryDetections.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  lidar_process
)
