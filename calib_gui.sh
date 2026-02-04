#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.023 --ros-args -r image:=/camera/image_raw -r camera:=/camera
