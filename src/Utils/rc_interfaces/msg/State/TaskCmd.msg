# TaskCmd.msg
# 서버로부터 받는 태스크 명령

std_msgs/Header header

int32[] route_ids         # 경유 마커 ID 리스트
int32 goal_id             # 최종 목표 마커 ID
string goal               # 최종 목표 (마커 ID 또는 슬롯명) - 문자열
string task_type          # 태스크 유형 (PICKUP, DROPOFF, RETURN 등)
string task_id            # 태스크 고유 ID
