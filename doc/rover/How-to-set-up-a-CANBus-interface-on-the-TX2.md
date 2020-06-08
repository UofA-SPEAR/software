# How to set up CAN on the TX2

Linux, and SPEAR, rely on a set of kernal modules originally developed by Volkswagen Engineering called "SocketCAN" for interfacing with the CAN network. [Refer to this Wikipedia page for info about it.](https://en.wikipedia.org/wiki/SocketCAN). Traditionally, CAN has worked by just sending characters to a file like `/dev/can0`, but this had the issue of only letting one process use it at a time (bad for literally anything!). SocketCAN alleviates this by setting up a network interface and extending the default Linux socket implementation to let any amount of processes send CAN frames at the same time.

We specifically use [UAVCAN](https://uavcan.org/), which defines a message protocol (similar to ROS's messages) so that you don't just have to manually send arbritrary bytes around and figure out which CAN frame belongs to which node. It's a good thing.

For debugging CAN, I (Johann) recommend using [`uavcan_gui_tool`](https://uavcan.org/GUI_Tool/Overview/). It should already be installed. If it isn't, go ~~yell at someone~~ [install it](#installing-uavcan-gui-tool).

## Using CAN without Docker

### For testing without anything connected

When testing without anything connected, you set up a virtual can interface, like this:

```shell
$ sudo modprobe can
$ sudo modprobe can_raw
$ sudo modprobe vcan
$ sudo ip link add dev vcan0 type vcan
$ sudo ip link set up vcan0
# Check to make sure it works (should show something similar to this):
$ ip -d -s -h -c link show vcan0
3: vcan0: <NOARP,UP,LOWER_UP> mtu 16 qdisc noqueue state UNKNOWN 
    link/can
```

Or, use the `setup-vcan.bash` script on the desktop.

### For using with physical CAN nodes hooked up

Pretty much the same as testing without anything connected, but you `modprobe can_dev` instead of `modprobe vcan`, and set a whole bunch of parameters:

```shell
$ sudo modprobe can_dev
$ sudo modprobe can
$ sudo modprobe can_raw
$ sudo modprobe mttcan
$ sudo ip link set can0 \
	type can \
	bitrate 500000 \
	sjw 4 \
	dbitrate 1000000 \
	dsjw 4 \
	berr-reporting on \
	fd on \
	restart-ms 1000
$ sudo ip link set up can0
# Check to make sure it works:
$ ip -d -s -h -c link show can0
```

Or, use the `setup-can.bash` script on the desktop.

#### Explanation of parameters:
- `type can`: Set type of interface to can
- `bitrate 500000`: Set bitrate to 500kbit, which all of our devices use
- `sjw 4`: Allow controller to adjust for frames arriving at slightly wrong baud rates
- `dbitrate 1000000`: Sets the bitrate for any nodes that might support CAN FD
- `dsjw 4`: Adjust for frames coming from CAN FD nodes
- `berr-reporting on`: Allows for bus error reporting. Useful for figuring out why things don't work.
- `fd on`: Enables CAN FD. Regular CAN nodes will still work with this though - CAN FD frames are backwards-compatible.
- `restart-ms 1000`: Allows the can0 interface to restart if everything is inactive. Helps for when the buffer is filled.

## Using CAN with Docker

[See this section in the README.md](https://github.com/UofA-SPEAR/software#run-with-canbus-support).

## Installing uavcan gui tool

Go to [the project page](https://uavcan.org/GUI_Tool/Overview/) for up-to-date steps, but as of 2020/01/27 the steps are:

```shell
$ sudo apt-get install -y python3-pip python3-setuptools python3-wheel
$ sudo apt-get install -y python3-numpy python3-pyqt5 python3-pyqt5.qtsvg git-core
$ sudo pip3 install git+https://github.com/UAVCAN/gui_tool@master
```

Open it by running `uavcan_gui_tool` at the command line.

I (Johann) personally recommend:

1) Set the `uavcan_gui_tool`'s node id by hitting the checkmark next to the "Set local node ID" box
2) Open the bus monitor by going Panels > Bus Monitor (or just hit `Ctrl`-`Shift`-`B`)
3) Click the bus monitor's "Start / stop capturing" button to show UAVCAN (and other CAN) messages
4) Have fun :grin:

## Generating a pdf file from this markdown document

```shell
$ pandoc "$HOME/Desktop/CANBus - How To.md" --from=markdown_github --latex-engine=xelatex -o "$HOME/Desktop/CANBus - How To.pdf"
```
