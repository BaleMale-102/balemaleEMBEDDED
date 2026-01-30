std_msgs/Header header
string task_id

# 선택: 서버가 기대하는 차량 번호판(검증용)
string plate_target

# 서버가 주는 경로: marker id 순서
int32[] route_ids

# 옵션: 목적지/슬롯/구역
string goal
