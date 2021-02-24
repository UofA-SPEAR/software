#!/usr/bin/env bash
#
# docker-with-can.bash
#
# Runs `docker-compose run spear` after modprobing the required kernel modules
# for CANBus use / debugging.
#

read -r -d '' help_msg <<'EOF'
docker-with-can.bash

Starts our docker environment via `docker-compose run spear` after loading
the kernel modules required for CANBus use / debugging.

By default, this script loads the modules necessary for actually using a
physical CANBus interface. If you want to use a virtual CANBus for local
debugging (i.e. not on the TX2), then pass the `--vcan` switch.

USAGE:
	docker-with-can.bash [[--help | -?] | --vcan]

ARGUMENTS:
	--help, -h		Print this help message
	--vcan			Setup for local, external-device-free work via a virtual CANBus
EOF

USE_VCAN="false"

# Color/text escape codes for pretty-printing:
RED=$(tput setaf 1)
YELLOW=$(tput setaf 3)
POWDER_BLUE=$(tput setaf 153)
WHITE=$(tput setaf 7)
NORMAL=$(tput sgr0)

print_error() {
	printf "${NORMAL}${BOLD}${WHITE}[${RED}ERROR${WHITE}]: %s\n${NORMAL}" "$1"
}

print_warning() {
  printf "${NORMAL}${BOLD}${WHITE}[${YELLOW}WARNING${WHITE}]: %s\n${NORMAL}" "$1"
}

print_info() {
  printf "${NORMAL}${BOLD}${WHITE}[${POWDER_BLUE}INFO${WHITE}]: %s\n${NORMAL}" "$1"
}

# Utility funciton for loading module with error checking
load_kernel_module() {
	print_info "Loading $1..."
	if ! sudo modprobe "$1"; then
		# If we get here, then modprobe returned a non-zero exit status.
		print_error "Error loading module ${1}!"
		exit 1
	fi
}

if [[ "$#" -eq "0" ]]; then   # No args passed, use physical CAN
	USE_VCAN="false"
elif [[ "$#" -eq "1" ]]; then # One arg passed
	case "$1" in
	"--help" | "-h")
		echo "$help_msg"
		exit 0
		;;

	"--vcan")
		USE_VCAN="true"
		;;

	*)
		print_error "Unknown argument passed!"
		echo ""
		echo "$help_msg"
		exit 1
		;;

	esac
else                          # Too many arguments passed!
	print_error "Too many arguments passed!"
	echo ""
	echo "$help_msg"
	exit 1
fi

if [[ "$USE_VCAN" = "true" ]]; then
	print_info "Setting up kernel modules for virtual CANBus..."
else
	print_info "Setting up kernel modules for physical CANBus..."
fi

print_warning "Must use sudo to load kernel modules!"

[[ "$USE_VCAN" = "false" ]] && load_kernel_module can_dev
load_kernel_module can
load_kernel_module can_raw
[[ "$USE_VCAN" = "false" ]] && load_kernel_module mttcan
[[ "$USE_VCAN" = "true" ]] && load_kernel_module vcan

print_info "Starting docker container..."
docker-compose run spear
