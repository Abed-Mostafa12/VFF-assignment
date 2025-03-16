cmake_minimum_required(VERSION 3.8)
project(vff_avoidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(avoidance_node src/avoidance_node.cpp)
ament_target_dependencies(avoidance_node rclcpp sensor_msgs geometry_msgs visualization_msgs)

install(TARGETS
  avoidance_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

