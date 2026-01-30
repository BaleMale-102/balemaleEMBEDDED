# LaneStatus.msg
# 차선 인식 결과

std_msgs/Header header

bool in_lane              # 차선 검출 여부
float32 offset_px         # 중심으로부터 오프셋 (픽셀)
float32 offset_norm       # 정규화된 오프셋 (-1 ~ +1, 왼쪽이 음수)
float32 angle             # 차선 각도 (rad, 반시계가 양수)
float32 quality           # 검출 품질 (0 ~ 1)
