std_msgs/Header header

# LANE_FOLLOW / MARKER_ACQUIRE / MARKER_SERVO / STOP_STABILIZE
# TURNING / POST_TURN_RECOVER / PAUSED / EMERGENCY_STOP
string state

int32 target_marker_id   # -1이면 없음 (route에서 next)
int32 observed_marker_id # 현재 사용 중인 검출 id (확정된 것)

float32 ex   # marker_in_base.x (가능하면, 아니면 0)
float32 ey   # marker_in_base.y
float32 eyaw # optional

float32 confidence  # 0..1 (marker 확정/추정 신뢰도)
string detail
