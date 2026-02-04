# Parking state status for debugging and monitoring
std_msgs/Header header

string sub_state              # Current parking sub-state (DETECT, RECOVERY, ALIGN_MARKER, etc.)
int32 detected_slot_id        # Currently detected slot marker ID (-1 if none)
int32 target_slot_id          # Target slot marker ID
bool slot_verified            # Slot group verification status
float32 marker_distance       # Distance to detected marker (m)
float32 marker_angle          # Angle to detected marker (rad)
string message                # Status message for debugging
