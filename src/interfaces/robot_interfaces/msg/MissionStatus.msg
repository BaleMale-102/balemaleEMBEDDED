# MissionStatus.msg
# 미션 상태 (로봇 → 서버)

std_msgs/Header header

string mission_id                       # 미션 고유 ID
string status                           # READY, RUNNING, PAUSED, COMPLETED, FAILED

int32 current_waypoint_index            # 현재 웨이포인트 인덱스
int32 total_waypoints                   # 전체 웨이포인트 수
float32 progress_percent                # 진행률 (0~100)

int32 current_marker_id                 # 현재 마커 ID
geometry_msgs/Pose current_pose         # 현재 위치 (추정)

string message                          # 상태 메시지
