# DrivingState.msg
# 주행 상태 (실시간 제어 상태 + 에러 정보)

std_msgs/Header header

string state              # 현재 제어 모드 (LANE_FOLLOW, TURNING, ALIGN_TO_MARKER, PARK_LINE_ALIGN 등)
int32 target_marker_id    # 목표 마커 ID (-1: 없음)
int32 observed_marker_id  # 현재 관측 마커 ID (-1: 없음)
float32 ex                # X 방향 에러 (m)
float32 ey                # Y 방향 에러 (m)
float32 eyaw              # Yaw 에러 (rad)
float32 confidence        # 상태 신뢰도 (0 ~ 1)
string detail             # 상세 정보 (디버그용)
