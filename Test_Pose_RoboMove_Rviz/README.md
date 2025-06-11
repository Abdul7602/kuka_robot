# my_moveit_pose

**A ROS 2 node for following target poses using MoveIt 2 and external pose tracking**

## Overview

This package provides a ROS 2 node that subscribes to an external pose (e.g., from a motion capture system) and sends the end-effector to that pose using MoveIt 2.  
It is designed for robots using custom joint and group names and can be adapted to your robot's configuration.

The main script is `track_motive_pose.py`, which listens to a `geometry_msgs/PoseStamped` message on `/marker_pose` and commands your robot's arm to follow the pose.

---

## Directory Structure

```
my_moveit_pose/
├── launch/
│   └── track_motive_pose.launch.py
└── my_moveit_pose/
    └── track_motive_pose.py
```

## Usage Example

### 1. Publish a Test Pose

Open a terminal and publish a test pose to `/marker_pose`:

```bash
ros2 topic pub /marker_pose geometry_msgs/PoseStamped "header:
  frame_id: 'lbr_link_0'
pose:
  position:
    x: 0.4
    y: 0.0
    z: 1.05
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### 2. Launch the Pose Follower Node

In your ROS workspace, launch the node:

```bash
ros2 launch my_moveit_pose track_motive_pose.launch.py
```

This will start the `track_motive_pose` node, which subscribes to `/marker_pose` and commands the robot using MoveIt 2.

### 3. (Optional) Launch Other Required Nodes

You may also wish to launch the following, depending on your setup:

- **Launch OptiTrack Bringup (for motion capture system integration):**

  ```bash
  ros2 launch combined_launch optitrack_bringup.launch.py
  ```

- **Launch Robot Mock Setup (for simulating/mock robot environment):**

  ```bash
  ros2 launch combined_launch robot_mock_setup.launch.py
  ```

---

## track_motive_pose.launch.py

This launch file starts the pose tracking node and handles topic remappings as needed for your robot:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_moveit_pose',
            executable='track_motive_pose',
            name='track_motive_pose',
            output='screen',
            remappings=[
                ('/joint_states', '/lbr/joint_states'),
                ('/plan_kinematic_path', '/lbr/plan_kinematic_path'),
            ]
        ),
    ])
```

---

## track_motive_pose.py

Core logic for subscribing to `/marker_pose` and calling MoveIt 2:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymoveit2 import MoveIt2

class MotivePoseFollower(Node):
    def __init__(self):
        super().__init__("track_motive_pose")
        self.moveit2 = MoveIt2(
            node=self,
            base_link_name="lbr_link_0",
            end_effector_name="lbr_link_ee",
            joint_names=[
                "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A6", "lbr_A1", "lbr_A5", "lbr_A7"
            ],
            group_name="arm"
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            "/marker_pose",
            self.pose_callback,
            1
        )
        self.get_logger().info("Motive pose follower node started.")

    def pose_callback(self, msg):
        self.get_logger().info(
            f"Received marker pose: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )
        try:
            result = self.moveit2.move_to_pose(msg.pose)
            self.get_logger().info(f"Sent move_to_pose command, result: {result}")
        except Exception as e:
            self.get_logger().error(f"Exception in move_to_pose: {e}")

def main():
    rclpy.init()
    node = MotivePoseFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

```bash
ros2 topic pub /marker_pose geometry_msgs/PoseStamped "header:
  frame_id: 'lbr_link_0'
pose:
  position:
    x: 0.4
    y: 0.0
    z: 1.05
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
working Test Poses
ubuntu@ubuntu:~/ros2_ws$ 
```bash

ros2 topic pub /marker_pose geometry_msgs/PoseStamped "header:
  frame_id: 'lbr_link_0'
pose:
  position:
    x: 0.0
    y: 0.3
    z: 1.2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707"
```

## Customization

- **Joint and group names:**  
  Update the `joint_names` and `group_name` in `track_motive_pose.py` to match your MoveIt 2 configuration.
- **Topic remapping:**  
  Adjust the remappings in the launch file if your robot uses different joint state or planning topics.

---

## Requirements

- ROS 2 (tested on Humble/Foxy/Galactic)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/source/)
- [pymoveit2](https://github.com/AndrejOrsula/pymoveit2)

---

## License

MIT

---

## Acknowledgements

- Built on top of MoveIt 2 and pymoveit2.
- Inspired by typical MoCap-to-arm demo setups.
