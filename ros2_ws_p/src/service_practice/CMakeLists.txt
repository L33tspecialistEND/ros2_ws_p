cmake_minimum_required(VERSION 3.8)
project(service_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(my_client src/my_client.cpp)
ament_target_dependencies(my_client rclcpp example_interfaces)

add_executable(my_server src/my_server.cpp)
ament_target_dependencies(my_server rclcpp example_interfaces)

add_executable(robot_arm_server src/robot_arm_server.cpp)
ament_target_dependencies(robot_arm_server rclcpp robot_interfaces)

add_executable(arm_position_client src/arm_position_client.cpp)
ament_target_dependencies(arm_position_client rclcpp robot_interfaces)

install(TARGETS 
  my_client
  my_server
  robot_arm_server
  arm_position_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
