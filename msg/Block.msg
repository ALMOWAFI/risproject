# Block message definition
# Represents a single colored block detected by vision system

std_msgs/Header header

# Block identifier (0-3 for 4 blocks)
int32 id

# Block color (as string: "red", "green", "blue", "yellow")
string color

# Block position in robot base frame (meters)
geometry_msgs/Point position

# Block orientation (if needed)
geometry_msgs/Quaternion orientation

# Confidence score (0.0 to 1.0)
float32 confidence

# Whether block is currently being held/interacted with
bool is_selected
