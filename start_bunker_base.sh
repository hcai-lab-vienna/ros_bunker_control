#!/bin/bash
source install/setup.bash

# enable kernel module: gs_usb
sudo modprobe gs_usb

# bring up can interface
sudo ip link set can0 up type can bitrate 500000

ros2 launch bunker_base bunker_base.launch.py
