cmake_minimum_required(VERSION 3.5)
project(noisy_odometry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(noisy_odometry_node src/noisy_odometry_node.cpp)
ament_target_dependencies(noisy_odometry_node rclcpp sensor_msgs nav_msgs tf2_ros)

install(TARGETS
  noisy_odometry_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
