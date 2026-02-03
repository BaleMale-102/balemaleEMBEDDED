# Lane detection result
std_msgs/Header header

bool valid                        # Lane detected
float32 offset                    # Offset from lane center (m, positive=right)
float32 offset_normalized         # Normalized offset (-1 to +1)
float32 angle                     # Lane angle (rad)
float32 curvature                 # Lane curvature (1/m)
float32 confidence                # Detection confidence (0-1)
