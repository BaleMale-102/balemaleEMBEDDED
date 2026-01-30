# MarkerStatus.msg
# ArUco 마커 인식 결과

std_msgs/Header header

bool valid                # 마커 검출 여부
int32 id                  # 마커 ID
float32 rel_x             # 카메라 기준 X (오른쪽 양수, m)
float32 rel_y             # 카메라 기준 Y (아래 양수, m)
float32 rel_z             # 카메라 기준 Z (전방 양수, m)
float32 rel_yaw           # 마커 yaw (rad)
float32 quality           # 검출 품질 (0 ~ 1)
