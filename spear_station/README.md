# Station 2
This is the ground control package. It contains all the tools nessesary for
controlling the rover. 

## Launch Files

### udp_station.launch

Launches nodes to send and receive ROS messages via the UDP protocol.
We send joystick messages from the station to the rover via UDP.
We also receive camera feeds from the rover via UDP.

### joystick.launch

Launches a `joy_node` node which publishes joystick positions to a ros topic.
We send these joystick messages to the rover where they are converted to drive commands.

## Nodes

### arm.py

Node which runs a Kivy arm interface.

### drive.py

Node that runs a Kivy driving interface.

### tmp_drive_joy_sender.py

VERY temporary node written for the SAR video. Just matches joystick commands straight
to left/right drive commands.


