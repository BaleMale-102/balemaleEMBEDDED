# Marker.msg
# 단일 ArUco 마커 정보

int32 id                           # 마커 ID
geometry_msgs/Pose pose            # 카메라 좌표계 기준 포즈

float32 distance                   # 마커까지 거리 (m)
float32 angle                      # 마커 방향 (rad, 양수=왼쪽)
float32 confidence                 # 검출 신뢰도 (0~1)
