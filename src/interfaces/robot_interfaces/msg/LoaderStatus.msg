# Loader status message
std_msgs/Header header

# Status constants
string STATUS_IDLE="IDLE"
string STATUS_LOADING="LOADING"
string STATUS_UNLOADING="UNLOADING"
string STATUS_DONE="DONE"
string STATUS_ERROR="ERROR"

string status               # Current loader status
bool is_loaded              # Whether vehicle is currently loaded
string message              # Status message or error description
