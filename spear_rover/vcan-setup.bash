#!/usr/bin/env bash
#
# Sets up a virtual CAN Bus interface on vcan0.

echo "[WARNING]: We're using sudo to enable vcan! Sorry!"

# Only use modprobe if not running inside docker!
if [ -z "${RUNNING_IN_DOCKER+x} " ]; then
  sudo modprobe vcan
else
  echo "Running in Docker. Going to skip using modprobe."
  echo ""
  echo "[NOTICE]: We're installing uavcan_gui_tool for you."
  echo "          We're also going to install a more recent python 3 version via"
  echo "          a PPA. uavcan_gui_tool use IPython for its interactive console,"
  echo "          which requires a more recent version of python 3 than what Ubuntu 16.04"
  echo "          includes in its repositories."
  echo ""

  sudo apt update
  sudo apt install -y software-properties-common # for the add-apt-repository command
  sudo add-apt-repository -y ppa:deadsnakes/ppa
  sudo apt update
  sudo apt install -y python3-pip \
                      python3.7 \
                      python3.7-{dev,venv,distutils,lib2to3,gdbm,tk} \
                      git-core
  sudo python3.7 -m pip install --upgrade pip setuptools wheel
  sudo python3.7 -m pip install --upgrade numpy pyqt5
  sudo python3.7 -m pip install --upgrade git+https://github.com/UAVCAN/gui_tool@master
fi

# Check if the ip command is installed (it usually isn't in Ubuntu's docker
# image).
command -v ip >/dev/null 2>&1 || {
  echo ""
  echo "[NOTICE]: The \`ip\` command is not installed."
  echo "          This usually happens when you're running Docker."
  echo "          We're going to install it now for you."
  echo ""
  sudo apt update
  sudo apt install -y iproute2
}

sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
