# This is a command for a single Actuator. It currently corresponds to the
# the equivalent UAVCAN type, uavcan.equipment.actuator.Command.
#
# We do it this way instead of just importing from canros because:
#
# - You can't require messages from canros without having canros trying to start
#   a CAN node, which can't be done on the base station.
# - This allows us to choose our own representation of actuators.

# The ID of the actuator. Used for things like CAN nodes deciding which UAVCAN
# message they want to listen to.
uint8 actuator_id

# The type of the actuator command
uint8 COMMAND_TYPE_UNITLESS = 0 # [-1, 1]
uint8 COMMAND_TYPE_POSITION = 1 # metres or radians
uint8 COMMAND_TYPE_FORCE    = 2 # Newtons or Newton-metres
uint8 COMMAND_TYPE_SPEED    = 3 # m/s or rad/s
uint8 command_type

float32 command_value
