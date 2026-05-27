cmake_minimum_required(VERSION 3.8)
project(g1_joint_state_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_go REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(g1_joint_state_bridge src/g1_joint_state_bridge.cpp)
ament_target_dependencies(g1_joint_state_bridge
  rclcpp sensor_msgs geometry_msgs tf2_ros unitree_hg unitree_go)

add_executable(g1_scene_publisher src/g1_scene_publisher.cpp)
ament_target_dependencies(g1_scene_publisher
  rclcpp visualization_msgs)

add_executable(g1_obstacle_detector src/g1_obstacle_detector.cpp)
ament_target_dependencies(g1_obstacle_detector
  rclcpp sensor_msgs tf2_ros tf2 geometry_msgs visualization_msgs std_msgs)

add_executable(g1_lidar_sim src/g1_lidar_sim.cpp)
ament_target_dependencies(g1_lidar_sim
  rclcpp sensor_msgs tf2_ros tf2 geometry_msgs)

install(TARGETS
  g1_joint_state_bridge
  g1_scene_publisher
  g1_obstacle_detector
  g1_lidar_sim
  DESTINATION lib/\${PROJECT_NAME})

ament_package()
