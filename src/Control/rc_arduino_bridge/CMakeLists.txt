cmake_minimum_required(VERSION 3.8)
project(arduino_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rc_interfaces REQUIRED)

# =========================
# Executable
# =========================
add_executable(arduino_bridge_node
  src/arduino_bridge_node.cpp
)

ament_target_dependencies(arduino_bridge_node
  rclcpp
  geometry_msgs
  rc_interfaces
)

# =========================
# Install
# =========================
install(TARGETS arduino_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
