# TaskStatus.msg
# 태스크 수행 상태

std_msgs/Header header

string task_id            # 태스크 고유 ID
string status             # 상태 (RUNNING, COMPLETED, FAILED, CANCELLED)
string current_state      # 현재 FSM 상태
int32 current_marker_id   # 현재 위치 마커 ID
float32 progress          # 진행률 (0 ~ 1)
string message            # 상태 메시지
