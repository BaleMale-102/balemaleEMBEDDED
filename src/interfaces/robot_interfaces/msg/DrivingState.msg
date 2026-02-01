# DrivingState.msg
# 현재 주행 상태 정보

std_msgs/Header header

# 상태 정보
string state                            # IDLE, DRIVING, TURNING, LINE_FOLLOWING, PARKING, ERROR
string sub_state                        # 세부 상태

# 마커 정보
int32 current_marker_id                 # 현재 가장 가까운 마커 (-1: 없음)
int32 target_marker_id                  # 목표 마커 (-1: 없음)
float32 distance_to_target              # 목표까지 거리 (m)

# 속도 정보
float32 linear_x                        # 현재 전진 속도 (m/s)
float32 linear_y                        # 현재 횡이동 속도 (m/s)
float32 angular_z                       # 현재 회전 속도 (rad/s)

# 추가 정보
string info                             # 디버그 정보
