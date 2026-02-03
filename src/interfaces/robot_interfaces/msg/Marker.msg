# Single ArUco marker detection result
std_msgs/Header header

int32 id                          # Marker ID
geometry_msgs/Pose pose           # 3D pose in camera frame
float32 distance                  # Distance from camera (m)
float32 angle                     # Horizontal angle from camera center (rad)
float32 confidence                # Detection confidence (0-1)
