# dosimeter_dummy_node #

This node publishes dummy dosimeter readings. This can be used for testing purposes.

This node assumes that a GPS fix is being published to `/gps/fix` since it uses
GPS coordinates to generate a fake dosimeter reading.

The instantaneous dose (Sv/hr) will be published to `/dosimeter/instantaneous`
The accumulated dose (Sv) will be published to `/dosimeter/accumulated`

