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
- 0.3 -1.4 0.6 = physical translation offset between the Motive origin and robot base
- 0 0 0 = no rotation needed (axes are already aligned via Z-up setting in Motive)

🦾 Robot Setup

The iiwa14.urdf.xacro describes:

- The KUKA iiwa14 robot   
- A wall and visual adapter    
- The robot base link (lbr_link_0) is fixed to the wall via a URDF joint

📍 TF Tree After Setup

map (from Motive)
 └── world (ROS)
     └── lbr_link_0 (robot base)
         └── ...
             └── visual_probe

🛠️ Notes

```
ros2 run my_moveit_pose track_motive_pose   --ros-args   --remap /joint_states:=/lbr/joint_states   --remap /plan_kinematic_path:=/lbr/plan_kinematic_path
```
    
- If you revert Motive to Y-up, you'll need to apply a rotation using but it does not work properly so stick to this z axis since rviz uses the same in ros world z axis is up:
```
    ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 map world
```
 - The current setup assumes Z-up streaming is active in Motive (Settings → Coordinate System).

🧪 Testing
    
- Launch RViz
- Set Fixed Frame to world
- Visualize TF, robot model, and Motive markers — they should align and move correctly in all 3D directions
