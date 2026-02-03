# Mission command from server
std_msgs/Header header

string command                    # Command type: START, STOP, PAUSE, RESUME
int32[] waypoint_ids              # List of marker IDs to visit
int32 final_goal_id               # Final destination marker ID
string task_type                  # PICKUP, DROPOFF, RETURN, etc.
string task_id                    # Unique task identifier