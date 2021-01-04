uint8 STATUS_UNKNOWN = 0
uint8 STATUS_RECOGNIZED = 1

float32 width
float32 height
float32 validity
geometry_msgs/Pose pose
geometry_msgs/Point center_2d

string label # Label if object is recognized
uint8 status
