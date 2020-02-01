#!/usr/bin/env bash
#
# Sets up a CAN Bus interface on can0.

# Color/text escape codes for pretty-printing:
BLACK=$(tput setaf 0)
RED=$(tput setaf 1)
GREEN=$(tput setaf 2)
YELLOW=$(tput setaf 3)
LIME_YELLOW=$(tput setaf 190)
POWDER_BLUE=$(tput setaf 153)
BLUE=$(tput setaf 4)
MAGENTA=$(tput setaf 5)
CYAN=$(tput setaf 6)
WHITE=$(tput setaf 7)
BRIGHT=$(tput bold)
NORMAL=$(tput sgr0)
BLINK=$(tput blink)
REVERSE=$(tput smso)
UNDERLINE=$(tput smul)

print_warning() {
  printf "${NORMAL}${BOLD}${WHITE}[${YELLOW}WARNING${WHITE}]: %s\n${NORMAL}" "$1"
}

print_info() {
  printf "${NORMAL}${BOLD}${WHITE}[${POWDER_BLUE}INFO${WHITE}]: %s\n${NORMAL}" "$1"
}

print_warning "We're using sudo to enable can! Sorry!"

print_info "Loading CAN kernel modules..."
sudo modprobe can_dev
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

print_info "Setting up can0 network interface..."

# Parameters explained:
#   type can ==> Set type of interface to can
#   bitrate 500000 ==> Set bitrate to 500kbit, which all of our devices use
#   sjw 4 ==> Allow controller to adjust for frames arriving at slightly wrong baud rates
#   dbitrate 1000000 ==> Sets the bitrate for any nodes that might support CAN FD
#   dsjw 4 ==> Adjust for frames coming from CAN FD nodes
#   berr-reporting on ==> Allows for bus error reporting. Useful for figuring out why things don't work.
#   fd on ==> Enables CAN FD. Regular CAN nodes will still work with this though - CAN FD frames are backwards-compatible.
#   restart-ms 1000 ==> Allows the can0 interface to restart if everything is inactive. Helps for when the buffer is filled.
sudo ip link set can0 \
  type can \
  bitrate 500000 \
  sjw 4 \
  dbitrate 1000000 \
  dsjw 4 \
  berr-reporting on \
  fd on \
  restart-ms 1000

sudo ip link set up can0

print_info "Here's the network that just got set up:\n"

# Flags explained:
#   -s ==> show statistics
#   -d ==> show details
#   -h ==> human-readable numbers
#   -c ==> colorize
ip -s -d -h -c link show can0

echo ""
read -n 1 -s -r -p "Press any key to continue..."
echo ""
