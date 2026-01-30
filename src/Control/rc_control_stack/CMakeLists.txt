cmake_minimum_required(VERSION 3.8)
project(rc_control_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rc_interfaces REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# =========================
# Executables
# =========================

# (1) Auto Control Stack
add_executable(control_stack_node
  src/control_stack_node.cpp
)
ament_target_dependencies(control_stack_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rc_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# (2) Safety + Emergency MUX (my code)
add_executable(safety_node
  src/safety_node.cpp
)
ament_target_dependencies(safety_node
  rclcpp
  rc_interfaces
  sensor_msgs
)

# =========================
# Install
# =========================
install(TARGETS
  control_stack_node
  safety_node
  DESTINATION lib/${PROJECT_NAME}
)

# config + launch 둘 다 설치 (launch 폴더 있으니 같이!)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
