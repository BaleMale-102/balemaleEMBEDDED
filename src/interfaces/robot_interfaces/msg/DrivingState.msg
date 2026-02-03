# Current driving state for diagnostics
std_msgs/Header header

string state                      # Current state (IDLE, LANE_FOLLOW, MARKER_APPROACH, etc.)
int32 target_marker_id            # Target marker ID (-1 if none)
int32 observed_marker_id          # Currently observed marker ID (-1 if none)
float32 error_x                   # X error (m)
float32 error_y                   # Y error (m)
float32 error_yaw                 # Yaw error (rad)
float32 cmd_vx                    # Commanded vx (m/s)
float32 cmd_vy                    # Commanded vy (m/s)
float32 cmd_wz                    # Commanded wz (rad/s)
string detail                     # Additional detail string
