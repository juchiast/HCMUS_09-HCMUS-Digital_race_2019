std_msgs/Header header

int32   x
int32   y
int32   width
int32   height
float32 angle       # Angle [0 to 2*PI), allow rotated rects

uint32                          id
string                          label
float32                         score   #Score as defined by the detection, Optional
std_msgs/ColorRGBA              color   # Define this object specific color
bool                            valid   # Defines if this object is valid, or invalid as defined by the filtering

sensor_msgs/Image               roi_image
