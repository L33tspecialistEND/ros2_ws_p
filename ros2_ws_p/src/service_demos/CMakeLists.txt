cmake_minimum_required(VERSION 3.8)
project(service_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(responsive_client_node src/responsive_client_node.cpp)
ament_target_dependencies(responsive_client_node rclcpp example_interfaces)

install(TARGETS 
  add_two_ints_client
  add_two_ints_server
  responsive_client_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
