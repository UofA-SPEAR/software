# rover2_can

This node maps UAVCAN messages to ROS messages, and vice-versa. It contains two
functions to facilitate this, documented [here](./docs/rover2_can.md). All of
the mappings themselves live within [`./nodes/mapper.py`](./nodes/mapper.py).
The utility functions live within the `src` directory, so that we can abstract
them away and you don't need to know exactly how they work to add new mappings.

## Adding new mappings

1) [_**Read the documentation!!!**_](./docs/rover2_can.md)
2) Add whatever mapping you want to the `nodes/mapper.py` node. 
3) Test the living heck out of everything.

## Testing locally (i.e. in a VM)
You can use a virtual CAN bus to facilitate testing this node. Run the
`vcan-setup.bash` script within this directory to set one up. I reccomend
downloading the official [UAVCAN GUI tool](https://uavcan.org/GUI_Tool/Overview/)
to send and recieve UAVCAN messages (the bus monitor is particuarly useful).

## Launch files
This node contains two distinct `roslaunch` launch files. You _**MUST**_ use
these to run this node, as the [`canros`](https://github.com/MonashUAS/canros)
package this node relies on uses a launch file to run a server. This node will
**not** work without that server being run, so please - use the launch files!

### `rover2_can.launch`
This is the base launch file, meant for actually running on the TX2. It listens
to UAVCAN messages on `can0` by default. This can be changed by passing in
parameters. For example, to listen on `can42` instead:

```shell
$ roslaunch rover2_can rover2_can.launch interface:=can42
```

### `development.launch`
This launch file is basically a quicker way to listen to `vcan0` (the interface
created by the virtual CAN bus setup script). All it does is include
`rover2_can.launch` and set `interface` to `vcan0`. Generally, this is the
launch file you'll want to use for testing in a VM.

## Building the documentation

```shell
$ # Run this after building everything with catkin_make:
$ pip2 install --user pydoc-markdown
$ pydocmd simple rover2_can+ rover2_can.util+ > docs/rover2_can.md
```
