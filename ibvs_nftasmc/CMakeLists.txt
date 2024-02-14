cmake_minimum_required(VERSION 3.0.2)
project(ibvs_nftasmc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

add_executable(video_publisher_node src/image_publisher.cpp)
target_link_libraries(video_publisher_node  ${catkin_LIBRARIES})
target_link_libraries(video_publisher_node  ${OpenCV_LIBRARIES})

add_executable(image_features_node src/image_features.cpp)
target_link_libraries(image_features_node  ${catkin_LIBRARIES})
target_link_libraries(image_features_node  ${OpenCV_LIBRARIES})

add_executable(attitude_nftasmc_node src/attitude_nftasmc.cpp)
target_link_libraries(attitude_nftasmc_node ${catkin_LIBRARIES})

add_executable(ibvs_pos_ctrl_node src/ibvs_pos_ctrl.cpp)
target_link_libraries(ibvs_pos_ctrl_node ${catkin_LIBRARIES})

add_executable(ibvs_pos_ctrl_node_VICON src/ibvs_pos_ctrl_VICON.cpp)
target_link_libraries(ibvs_pos_ctrl_node_VICON ${catkin_LIBRARIES})

add_executable(attitude_nftasmc_node_VICON src/attitude_nftasmc_VICON.cpp)
target_link_libraries(attitude_nftasmc_node_VICON ${catkin_LIBRARIES})

add_executable(uav_dynamics_node src/uav_dynamics.cpp)
target_link_libraries(uav_dynamics_node ${catkin_LIBRARIES})

add_executable(target_pos_node src/target_position.cpp)
target_link_libraries(target_pos_node ${catkin_LIBRARIES})

add_executable(target_broadcaster_node src/target_gazebo_broadcaster.cpp)
target_link_libraries(target_broadcaster_node ${catkin_LIBRARIES})

add_executable(quad_broadcaster_node src/quad_gazebo_broadcaster.cpp)
target_link_libraries(quad_broadcaster_node ${catkin_LIBRARIES})

add_executable(disturbances_node src/disturbances.cpp)
target_link_libraries(disturbances_node ${catkin_LIBRARIES})
