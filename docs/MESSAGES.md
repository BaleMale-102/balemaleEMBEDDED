# 커스텀 메시지 정의 (robot_interfaces)

## Marker.msg
```
std_msgs/Header header
int32 id                  # 마커 ID
geometry_msgs/Pose pose   # 3D 포즈
float32 distance          # 거리 (m)
float32 angle             # 각도 (rad)
float32 confidence        # 신뢰도 (0~1)
```

## MarkerArray.msg
```
std_msgs/Header header
Marker[] markers
```

## TrackedMarker.msg
```
std_msgs/Header header
int32 id
geometry_msgs/Pose pose
geometry_msgs/Pose predicted_pose
geometry_msgs/Vector3 velocity
float32 distance
float32 angle
bool is_detected
float32 prediction_confidence
float32 tracking_duration
float32 time_since_detection
```

## DrivingState.msg
```
string state              # IDLE, DRIVE, TURN 등
int32 target_marker_id
int32 observed_marker_id
float32 error_x, error_y, error_yaw
float32 cmd_vx, cmd_vy, cmd_wz
string detail
```

## MissionCommand.msg
```
string command            # START, STOP, REROUTE
int32[] waypoint_ids
int32 final_goal_id
string task_type          # PARK (입고), EXIT (출차), REROUTE
string task_id
```

### task_type별 플로우
- **PARK**: DRIVE → PARK_* → UNLOAD → RETURN_HOME (홈에서 적재 후 슬롯에 하역)
- **EXIT**: DRIVE → PARK_* → LOAD → RETURN_HOME → UNLOAD (슬롯에서 적재 후 홈에 하역)

## MissionStatus.msg
```
string task_id
string status             # RUNNING, COMPLETED, FAILED
string current_state
int32 current_waypoint_idx
int32 current_marker_id
float32 progress
string message
```

## Detection.msg
```
uint8 CLASS_PLATE=0, CLASS_STICKER=1, CLASS_PERSON=2, CLASS_BOX=3, CLASS_CONE=4
uint8 class_id
string class_name
float32 confidence
int32 x1, y1, x2, y2
string text               # OCR 결과
bool has_sticker
```

## DetectionArray.msg
```
std_msgs/Header header
Detection[] detections
uint8 num_plates
uint8 num_obstacles
```

## SlotLineStatus.msg
```
std_msgs/Header header
bool valid
float32 center_offset_x, center_offset_y
float32 angle, width, height, confidence
```

## ParkingStatus.msg
```
std_msgs/Header header
string sub_state
int32 detected_slot_id, target_slot_id
bool slot_verified
float32 marker_distance, marker_angle
string message
```

## LoaderCommand.msg
```
std_msgs/Header header
string command            # LOAD, UNLOAD, STOP
```

## LoaderStatus.msg
```
std_msgs/Header header
string status             # IDLE, LOADING, UNLOADING, DONE, ERROR
bool is_loaded
string message
```

## PlateQuery.msg / PlateResponse.msg
```
# Query
string plate_number
string car_id

# Response
string plate_number
bool verified
int32 assigned_slot_id
int32[] waypoint_ids
string message
```

---

## 서비스

### SetTargetMarker.srv
```
int32 marker_id → bool success, string message
```

### GetRobotState.srv
```
→ string state, int32 current_marker_id, float32 x/y/yaw, float32 battery_voltage
```

### SetRoute.srv
```
int32[] marker_ids → bool success, string message
```
