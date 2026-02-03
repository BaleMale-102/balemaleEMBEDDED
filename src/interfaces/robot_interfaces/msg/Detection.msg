# Detection.msg - 단일 객체 검출 결과
# ANPR 및 장애물 검출에 사용

# 클래스 상수
uint8 CLASS_PLATE=0
uint8 CLASS_STICKER=1
uint8 CLASS_PERSON=2
uint8 CLASS_BOX=3
uint8 CLASS_CONE=4

# 필드
uint8 class_id              # 검출된 객체 클래스 (위 상수 참조)
string class_name           # 클래스 이름 ("plate", "sticker", "person", "box", "cone")
float32 confidence          # 검출 신뢰도 (0~1)

# Bounding box (픽셀 좌표)
int32 x1                    # 좌상단 x
int32 y1                    # 좌상단 y
int32 x2                    # 우하단 x
int32 y2                    # 우하단 y

# 추가 정보 (OCR 결과 등)
string text                 # OCR 결과 (번호판인 경우)
bool has_sticker            # 스티커 유무 (번호판인 경우)
