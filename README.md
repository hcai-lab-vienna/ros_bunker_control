# ROS2 Bunker Control


## Setup

### Download

Requirments are [ROS2 jazzy](https://docs.ros.org/en/jazzy/Installation.html) (newer should also work) and:
```bash
sudo apt install build-essential git cmake libasio-dev can-utils
```

First clone repo with all submodules:
```bash
git clone --recurse-submodules https://github.com/hcai-lab-vienna/ros_bunker_control.git
```
or if you cloned the repo befor reading this, download all submodules like this:
```bash
git submodule update --init --remote --recursive
```
or follow the instruction from [agilexrobotics/bunker_ros2](https://github.com/agilexrobotics/bunker_ros2) for `src`.

---

Also init CAN2USB adapter once with, if not done before:
```bash
bash src/ugv_sdk/scripts/setup_can2usb.bash
```

### Build

In the root directory of the repository after downloading all submodules with an active ros environment do:
```bash
colcon build
```

For troubleshooting see:
- [agilexrobotics/bunker_ros2](https://github.com/agilexrobotics/bunker_ros2)
- [agilexrobotics/ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)


## Run

Connect Bunker via CAN2USB adapter and then do in the root of the repo after building:
```bash
source install/setup.bash
bash src/ugv_sdk/scripts/bringup_can2usb_500k.bash
ros2 launch bunker_base bunker_base.launch.py
```
or just run the `start_bunker_base.sh` script.
