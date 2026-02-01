# LaneStatus.msg
# 차선/라인 검출 결과

std_msgs/Header header

bool valid                              # 라인 검출 여부
float32 offset                          # 중심 오프셋 (m, 양수=오른쪽)
float32 offset_normalized               # 정규화된 오프셋 (-1~+1)
float32 angle                           # 라인 각도 (rad)
float32 curvature                       # 곡률 (1/m)
float32 confidence                      # 검출 신뢰도 (0~1)
