cmake_minimum_required(VERSION 3.8)
project(pavbot_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pavbot_imu src/pavbot_imu.cpp)

ament_target_dependencies(
  pavbot_imu
  rclcpp
  std_msgs
)

install(
  TARGETS pavbot_imu
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
