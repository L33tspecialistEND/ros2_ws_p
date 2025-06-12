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

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
