# Notes about ros_control #

Look [here](https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot) for some good info.

Essentially this replaces the hardware_interface_node with a more standard ROS interface
(read: doesn't require as much custom work and is more portable).

From this tutorial, don't use the ceperate ROBOTcpp format (yet anyways)

## Controllers

velocity_controllers/JointVelocityController

If you use the effort_controllers, it will require a PID loop, etc.

These controllers are really just pre-defined classes to inherit from that provide a consistent interface. You will want to implement the actual control stuff from here. In our case, this means interacting with CAN and the such.

Make sure you update all the controllers you try and spawn in the launch file.

## Transmissions

Essentially you have joints that can go through transmissions, which are
analogs for actual transmissions. You would use this if you were implementing
all the actual control stuff in ROS, instead of the embedded harwdware.
