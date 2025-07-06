# Combined Launch for OptiTrack and KUKA iiwa14 Mock Setup

This repository provides launch files for running:
- The **OptiTrack motion capture system** (via `optitrack_bringup.launch.py`)
- The **mock setup of the KUKA iiwa14 robot** with RViz visualization (via `robot_mock_setup.launch.py`)

## Prerequisites

Make sure you have:
- ROS 2 (Humble or compatible) installed
- All necessary packages for:
  - OptiTrack integration (e.g., `mocap4r2_marker_viz`)
  - KUKA LBR iiwa robot (e.g., `lbr_bringup`, MoveIt configuration)
- Proper environment setup (`source install/setup.bash`)

## Launch Instructions

### 1. Launch the OptiTrack System

This command starts nodes responsible for motion capture marker visualization using OptiTrack.

```
ros2 launch combined_launch optitrack_bringup.launch.py
```
This will bring up nodes related to the OptiTrack system and publish marker transforms.

### 2. Launch the KUKA iiwa14 Mock Setup with RViz

This command launches the iiwa14 robot mock setup including RViz with the MoveIt planning interface.
```
ros2 launch combined_launch robot_mock_setup.launch.py
```
This sets up a mock robot state publisher and RViz visualization for interacting with the iiwa14 robot.



ubuntu@ubuntu:~/
```
ros2 run tf2_ros static_transform_publisher \
  0 0 0  0.7071 0.0 0.7071 0.0 \
  world lbr_link_0
```

[WARN] [1751560908.134766300] []: Old-style arguments are deprecated; see --help for new-style arguments
[INFO] [1751560908.151590095] [static_transform_publisher_wv8UHSXXtuQEgntR]: Spinning until stopped - publishing transform
translation: ('0.000000', '0.000000', '0.000000')
rotation: ('0.707100', '0.000000', '0.707100', '0.000000')
from 'world' to 'lbr_link_0'

