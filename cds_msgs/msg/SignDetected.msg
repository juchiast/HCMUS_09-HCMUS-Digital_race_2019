std_msgs/Header header

# Sign id
# 0: Slow
# 1: Left
# 2: Right
int8 id

# Detected confident [0 - 1]
float32 confident

# Bounding box
int32 x
int32 y
int32 width
int32 height
