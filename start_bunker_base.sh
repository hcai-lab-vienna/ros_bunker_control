#!/bin/bash
source install/setup.bash
bash src/ugv_sdk/scripts/bringup_can2usb_500k.bash
ros2 launch bunker_base bunker_base.launch.py
