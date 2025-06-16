cmake_minimum_required(VERSION 3.8)
project(topic_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add include directory
include_directories(
  include
  ${ament_cmake_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces)
find_package(builtin_interfaces)

add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher rclcpp std_msgs)

add_executable(int_subscriber src/int_subscriber.cpp)
ament_target_dependencies(int_subscriber rclcpp std_msgs)

add_executable(robot_status_publisher src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher rclcpp robot_interfaces)

add_executable(random_int_publisher src/random_int_publisher.cpp)
ament_target_dependencies(random_int_publisher rclcpp std_msgs)
target_include_directories(random_int_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS 
  my_publisher
  random_int_publisher
  int_subscriber
  robot_status_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
