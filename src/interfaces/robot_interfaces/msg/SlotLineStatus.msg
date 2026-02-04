# Slot line (yellow rectangle) detection status
std_msgs/Header header

bool valid                    # Rectangle detection valid
float32 center_offset_x       # Rectangle center X offset from image center (m)
float32 center_offset_y       # Rectangle center Y offset from image center (m)
float32 angle                 # Rectangle rotation angle (rad)
float32 width                 # Detected rectangle width (pixels)
float32 height                # Detected rectangle height (pixels)
float32 confidence            # Detection confidence (0-1)
