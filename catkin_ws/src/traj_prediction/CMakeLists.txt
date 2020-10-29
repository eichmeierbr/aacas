cmake_minimum_required(VERSION 2.8.3)
project(traj_prediction)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs 
  yolov3_sort 
  rviz_visual_tools
  std_msgs
  geometry_msgs
  message_generation
  nav_msgs
)   



add_message_files(
    FILES
    tracked_obj.msg
    tracked_obj_arr.msg
    coeff.msg
    tracked_obj_coeff.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_process
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp  message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


## Declare a C++ library
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(traj_prediction_node src/traj_prediction.cpp)
add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})
target_link_libraries(traj_prediction_node ${catkin_LIBRARIES})

