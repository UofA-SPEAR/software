# Standard ROS message type for wheel stuff.

# Select wheel
uint8 wheel
# Note that the UAVCAN message is actually a float16, so this data could be truncated.
float32 velocity