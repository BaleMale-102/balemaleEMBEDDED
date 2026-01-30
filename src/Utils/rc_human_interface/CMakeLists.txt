cmake_minimum_required(VERSION 3.8)
project(rc_human_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rc_interfaces REQUIRED)

add_executable(keyboard_teleop_node src/keyboard_teleop_node.cpp)
ament_target_dependencies(keyboard_teleop_node
  rclcpp
  std_msgs
  rc_interfaces
)

install(TARGETS keyboard_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
