# MissionCommand.msg
# 미션 명령 (서버 → 로봇)

std_msgs/Header header

string command                          # START, STOP, PAUSE, RESUME
int32[] route                           # 마커 ID 시퀀스
string mission_type                     # DELIVERY, PATROL, CUSTOM
string mission_id                       # 미션 고유 ID
