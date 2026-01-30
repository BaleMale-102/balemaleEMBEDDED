std_msgs/Header header

bool in_lane

# 이미지 기준 오프셋 (px): +면 이미지 오른쪽, -면 왼쪽
float32 offset_px

# 정규화 오프셋 [-1..1] 권장 (0이 중앙)
float32 offset_norm

# 라인 각도(rad): 0이면 "수직(위아래) 기준선"에 가깝다고 가정
# (필요시 나중에 정의 바꿔도 됨. 지금은 parking_controller가 0으로 맞추는 목표값)
float32 angle

# 0..1 품질(신뢰도). safety_manager가 stale/quality 기반 stop 최종 처리하므로
# 여기서는 “좋을수록 1”로만 일관되게 제공
float32 quality