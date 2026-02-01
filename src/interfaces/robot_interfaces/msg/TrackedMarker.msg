# TrackedMarker.msg
# Kalman 필터로 추적된 마커 (예측 포함)

std_msgs/Header header

int32 id                                # 마커 ID
geometry_msgs/Pose pose                 # 현재/예측 포즈
geometry_msgs/Pose predicted_pose       # 예측 포즈 (참고용)
geometry_msgs/Vector3 velocity          # 추정 속도 (m/s)

float32 distance                        # 마커까지 거리 (m)
float32 angle                           # 마커 방향 (rad)

bool is_detected                        # 실제 검출 vs 예측
float32 prediction_confidence           # 예측 신뢰도 (0~1)
float32 tracking_duration               # 추적 지속 시간 (s)
float32 time_since_detection            # 마지막 검출 이후 시간 (s)
