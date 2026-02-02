cmake_minimum_required(VERSION 3.8)
project(imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  imu_driver/imu_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_node
)

ament_package()
