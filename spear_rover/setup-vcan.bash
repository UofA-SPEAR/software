#!/usr/bin/env bash
#
# Sets up a virtual CAN Bus interface on vcan0 for local, device-free CAN debugging.

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

print_warning "We're using sudo to enable vcan! Sorry!"

print_info "Loading virtual CAN kernel modules..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

print_info "Setting up can0 network interface..."

sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

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
