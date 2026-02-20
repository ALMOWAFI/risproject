# Player selection message
# Published when player selects/interacts with a block

std_msgs/Header header

# Selected block ID
int32 block_id

# Block color
string color

# Selection type: "pointed", "held", "touched"
string selection_type

# Timestamp of selection
time selection_time

# Confidence of selection detection
float32 confidence
