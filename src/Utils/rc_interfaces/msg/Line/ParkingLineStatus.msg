# ParkingLineStatus.msg
# 주차선 인식 결과

std_msgs/Header header

bool valid                # 주차선 검출 여부
float32 offset_norm       # 정규화된 오프셋 (-1 ~ +1)
float32 angle             # 주차선 각도 (rad)
float32 quality           # 검출 품질 (0 ~ 1)
