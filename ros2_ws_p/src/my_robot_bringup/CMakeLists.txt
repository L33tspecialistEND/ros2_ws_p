cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Will install launch directory when package is built
install(DIRECTORY
  launch      # This directory contains the launch file
  config      # This directory contains the parameter YAML files
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
