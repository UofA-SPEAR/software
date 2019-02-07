#!/usr/bin/env bash
#
# Sets up a virtual CAN Bus interface on vcan0.

echo "[WARNING]: We're using sudo to enable vcan! Sorry!"

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
