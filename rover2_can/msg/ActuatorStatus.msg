#
# Generic actuator feedback, if available.
#

Header header

uint8 actuator_id

# Whether the units are linear or angular depends on the actuator type (refer
# to the ActuatorCommand message type).
float32 position # in metres or radians
float32 force    # in Newtons or Newton-metres
float32 speed    # in metres per second or radians per second

uint8 POWER_RATING_PCT_UNKNOWN=127
uint8 power_rating_pct # 0 -> unloaded, 100 -> fully loaded.
