# TODO

## 빌드 및 실행

```bash
# 빌드 (메인 시스템)
cd ~/balemaleEMBEDDED
colcon build
source install/setup.bash

# 전체 실행
ros2 launch robot_bringup system.launch.py

# 서버테스트 실행
ros2 launch robot_bringup server_test.launch.py

# 옵션
ros2 launch robot_bringup system.launch.py simulation:=true
ros2 launch robot_bringup system.launch.py show_debug:=false
ros2 launch robot_bringup system.launch.py cam_front_dev:=/dev/video1
```

### ANPR 노드 별도 실행 (conda anpr_310 환경)
```bash
# 터미널 2에서 실행
conda activate anpr_310
cd ~/balemaleEMBEDDED

# 빌드 (처음 한번, setuptools 58.2.0 필요)
pip install setuptools==58.2.0
colcon build --packages-select anpr_detector

# 실행
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run anpr_detector detector_node --ros-args -p show_debug_window:=true
```

### Launch 인자
| 인자 | 기본값 | 설명 |
|------|--------|------|
| simulation | false | 시뮬레이션 모드 |
| show_debug | true | 디버그 창 표시 |
| cam_front_dev | /dev/video0 | 전방 카메라 (C920) |
| cam_side_dev | /dev/video2 | 측면 카메라 (Brio 100) |
| arduino_port | /dev/ttyUSB0 | 모터 Arduino 포트 |
| loader_port | /dev/ttyUSB1 | Loader Arduino 포트 |

---

## 디버깅 명령

```bash
# 토픽 확인
ros2 topic list
ros2 topic echo /perception/tracked_marker
ros2 topic echo /perception/anpr/detections
ros2 topic echo /perception/anpr/plate
ros2 topic echo /driving/state
ros2 topic echo /mission/state

# ANPR 디버그 이미지 확인
ros2 run rqt_image_view rqt_image_view /perception/anpr/debug_image

# 테스트 미션 (마커 1 → 2)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START 1,2'"

# 주차 테스트 (슬롯 17)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START_PARK 17'"

# 출차 테스트 (슬롯 17에서 차량 회수)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'EXIT 1,5 17'"

# 주차 디버그
ros2 topic echo /parking/status
ros2 topic echo /perception/side_markers
ros2 run rqt_image_view rqt_image_view /slot_line_detector/debug_image

# 로더 테스트
ros2 topic echo /loader/status
ros2 topic pub --once /loader/command robot_interfaces/LoaderCommand "{command: 'LOAD'}"
ros2 topic pub --once /loader/command robot_interfaces/LoaderCommand "{command: 'UNLOAD'}"

# 풀 미션 디버그
ros2 topic echo /plate/query
ros2 topic echo /plate/response

# 긴급 정지
ros2 topic pub /control/cmd_vel geometry_msgs/Twist "{}"

# Arduino 직접 테스트
ros2 topic pub /control/cmd_vel geometry_msgs/Twist "{linear: {x: 0.005}}"
```

### 수동 제어 (teleop.py)
```bash
# Direct 모드 (기본) - ROS 없이 Arduino 직접 제어
python3 teleop.py

# ROS 모드 - ROS 노드 통해 제어
python3 teleop.py --ros

# 포트 지정
python3 teleop.py --port /dev/ttyUSB0