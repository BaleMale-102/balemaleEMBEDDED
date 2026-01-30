cmake_minimum_required(VERSION 3.8)
project(rc_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rc_interfaces REQUIRED)

# yaml-cpp (system dependency)
find_package(yaml-cpp REQUIRED)

add_executable(mission_manager_node
  src/mission_manager_node.cpp
)

ament_target_dependencies(mission_manager_node
  rclcpp
  std_msgs
  sensor_msgs
  rc_interfaces
)

target_link_libraries(mission_manager_node
  yaml-cpp
)

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
