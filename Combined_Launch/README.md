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


# 📡 KUKA iiwa14 + Motive Motion Capture Integration (ROS 2)

This setup integrates a **KUKA iiwa14 robot** URDF in ROS 2 with **OptiTrack Motive** motion capture data using a correct TF alignment between the Motive `map` frame and the ROS `world` frame.

---

## ✅ Why This Works

Motive originally streams motion capture data in a **Y-up coordinate system**, which **does not align with ROS**, where **Z is up**.

To make integration seamless:
- We changed **Motive's coordinate system** to stream with **Z-up**.
- This aligns all three axes directly with ROS standards.

| Axis Meaning        | Motive (After Change) | ROS Convention |
|---------------------|------------------------|----------------|
| Forward             | +X                     | +X ✅           |
| Left/Right          | +Y                     | +Y ✅           |
| **Up/Down**         | **+Z**                 | **+Z ✅**       |

---

## 🔄 Transform Between `map` and `world`

We publish a static transform to bridge `map` (from Motive) to `world` (used by URDF/TF):

```bash
ros2 run tf2_ros static_transform_publisher 0.3 -1.4 0.6 0 0 0 map world
```
