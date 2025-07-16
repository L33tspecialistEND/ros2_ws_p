cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # For generating interface code
find_package(builtin_interfaces REQUIRED)

# Specify message files here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg" 
  "srv/SetArmPosition.srv"
  "action/CountUntil.action"
  "action/CleanArea.action"
  "srv/ResetCounter.srv"
  "srv/ActivateTurtle.srv"
  DEPENDENCIES std_msgs builtin_interfaces  # List message dependencies here

  # Add other .msg, .srv, .action files if I make more
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  builtin_interfaces
)

ament_package()