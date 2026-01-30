# TaskStatus
std_msgs/Header header
string task_id

# IDLE / PLATE_CHECK / LOAD / DRIVE / PARK / UNLOAD / FINISH / ERROR
string state

int32 current_marker_id # 여기서
int32 next_marker_id    # 요기로 이동중

float32 progress_0_1    # 현재 edge에서 어느정도 진행됐는지 (안될수도있음)
string detail
