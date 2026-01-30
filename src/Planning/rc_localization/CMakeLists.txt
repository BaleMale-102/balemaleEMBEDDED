cmake_minimum_required(VERSION 3.8)
project(rc_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)          
find_package(tf2_ros REQUIRED)           
find_package(rc_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(localization_node src/localization_node.cpp)
ament_target_dependencies(localization_node
  rclcpp
  geometry_msgs
  std_msgs            
  tf2_ros             
  rc_interfaces
  sensor_msgs
)
target_link_libraries(localization_node yaml-cpp)

install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
