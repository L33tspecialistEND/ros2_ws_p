cmake_minimum_required(VERSION 3.8)
project(executor_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(minimal_executor_node src/minimal_executor_node.cpp)
ament_target_dependencies(minimal_executor_node rclcpp)

add_executable(explicit_single_threaded_node src/single_threaded_executor_node.cpp)
ament_target_dependencies(explicit_single_threaded_node rclcpp)

add_executable(executor_with_timer_node src/executor_with_timer_node.cpp)
ament_target_dependencies(executor_with_timer_node rclcpp)

install(TARGETS
  minimal_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  explicit_single_threaded_node
  DESTINATION lib/${PROJECT_NAME})

install (TARGETS 
  executor_with_timer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
