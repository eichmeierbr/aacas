cmake_minimum_required(VERSION 2.8.3)
project(dji_m600_sim)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  tf
  roscpp
  robot_state_publisher
  rviz_visual_tools
  message_generation
  lidar_process
)

add_service_files(
  DIRECTORY srv
  FILES
  SimDroneTaskControl.srv
#  QueryDetections.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

link_directories(
	${catkin_LIBRARY_DIRS}
)


add_executable(rviz_collision_publisher_no_gazebo src/rviz_collision_publisher_no_gazebo.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(rviz_collision_publisher_no_gazebo ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(rviz_collision_publisher_no_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
