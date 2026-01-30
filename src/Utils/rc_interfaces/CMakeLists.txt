cmake_minimum_required(VERSION 3.8)
project(rc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  # Line
  "msg/Line/LaneStatus.msg"
  "msg/Line/ParkingLineStatus.msg"
  # Marker
  "msg/Marker/MarkerStatus.msg"
  # State
  "msg/State/DriveCmd.msg"
  "msg/State/DrivingState.msg"
  "msg/State/TaskCmd.msg"
  "msg/State/TaskStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
