# Mission execution status
std_msgs/Header header

string task_id                    # Task identifier
string status                     # RUNNING, COMPLETED, FAILED, CANCELLED
string current_state              # Current FSM state
int32 current_waypoint_idx        # Current waypoint index in list
int32 current_marker_id           # Current marker location
float32 progress                  # Overall progress (0-1)
string message                    # Status message for logging
