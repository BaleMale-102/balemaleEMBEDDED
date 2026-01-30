cmake_minimum_required(VERSION 3.8)
project(rc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ROS2 interface generator
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =========================
# Message files
# =========================
set(msg_files
  "msg/DispatchRequest.msg"
  "msg/DispatchResponse.msg"
  "msg/ServerCmd.msg"

  "msg/Line/LaneStatus.msg"
  "msg/Line/ParkingLineStatus.msg"

  "msg/Marker/MarkerDetection.msg"
  "msg/Marker/MarkerDetections.msg"
  "msg/Marker/MarkerStatus.msg"

  "msg/State/DriveCmd.msg"
  "msg/State/DrivingEvent.msg"
  "msg/State/DrivingState.msg"
  "msg/State/TaskCmd.msg"
  "msg/State/TaskStatus.msg"
)

# =========================
# Generate interfaces
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# =========================
# Export runtime deps
# =========================
ament_export_dependencies(rosidl_default_runtime)

ament_package()
