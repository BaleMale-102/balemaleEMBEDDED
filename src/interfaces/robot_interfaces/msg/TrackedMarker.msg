# Tracked marker with Kalman filter prediction
std_msgs/Header header

int32 id                          # Marker ID being tracked
geometry_msgs/Pose pose           # Current/filtered pose
geometry_msgs/Pose predicted_pose # Predicted pose (0.5s ahead)
geometry_msgs/Vector3 velocity    # Estimated velocity (m/s)
float32 distance                  # Current distance (m)
float32 angle                     # Current angle (rad) - marker position relative to camera center
float32 marker_yaw                # Marker's own orientation/yaw (rad) - for heading alignment
bool is_detected                  # Currently detected
float32 prediction_confidence     # Prediction confidence (0-1)
float32 tracking_duration         # How long we've been tracking (s)
float32 time_since_detection      # Time since last detection (s)
