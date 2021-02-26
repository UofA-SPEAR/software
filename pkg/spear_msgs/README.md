# SPEAR messages #

This package is for ROS packages managed by SPEAR.

# Conventions #

.msg files need to be self-documenting. If you can't figure out what the message
does, or is for by reading the .msg file, you need to add more comments.
Comments are free, no reason to be sparse with them.

## Naming

All internal ROS messages shall be named using lowercase and underscores.
For example:

```
example_ros_message.msg
```

Any message that is used only to communicate with the CAN system should follow the UAVCAN
message naming standard, i.e.

```
ExampleRosMessage.msg
```
