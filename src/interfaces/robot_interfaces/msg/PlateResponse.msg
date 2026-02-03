# Plate verification response from server
std_msgs/Header header

string plate_number         # Queried plate number
bool verified               # Whether plate is in entry list
int32 assigned_slot_id      # Assigned parking slot ID (-1 if not verified)
int32[] waypoint_ids        # Waypoints to reach the slot
string message              # Response message
