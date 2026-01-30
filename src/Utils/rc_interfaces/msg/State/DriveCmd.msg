# DriveCmd.msg
# 주행 명령

std_msgs/Header header

bool enable               # 주행 활성화
float32 vx                # 전진 속도 (m/s, 전방 양수)
float32 vy                # 횡방향 속도 (m/s, 왼쪽 양수)
float32 wz                # 회전 속도 (rad/s, 반시계 양수)
string source             # 명령 출처 (디버그용)
