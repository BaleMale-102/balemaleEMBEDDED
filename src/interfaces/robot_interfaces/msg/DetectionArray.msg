# DetectionArray.msg - 검출 결과 배열
# ANPR 및 장애물 검출 결과를 담는 배열

std_msgs/Header header
Detection[] detections      # 검출된 객체 배열

# 요약 정보
uint8 num_plates            # 검출된 번호판 수
uint8 num_obstacles         # 검출된 장애물 수 (person + box + cone)
