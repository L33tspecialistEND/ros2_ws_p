cmake_minimum_required(VERSION 3.8)
project(action_package_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(count_up_to_server src/count_up_to_server_minimal.cpp)
ament_target_dependencies(count_up_to_server rclcpp rclcpp_action robot_interfaces)

add_executable(count_up_to_client src/count_up_to_client_minimal.cpp)
ament_target_dependencies(count_up_to_client rclcpp rclcpp_action robot_interfaces)

add_executable(count_up_to_client_full src/count_up_to_client.cpp)
ament_target_dependencies(count_up_to_client_full rclcpp rclcpp_action robot_interfaces)

add_executable(count_up_to_server_full src/count_up_to_client.cpp)
ament_target_dependencies(count_up_to_server_full rclcpp rclcpp_action robot_interfaces)

install(TARGETS
  count_up_to_server
  count_up_to_client
  count_up_to_client_full
  count_up_to_server_full
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()