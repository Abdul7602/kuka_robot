# Marker Pose Publisher & Streaming ID Setup

This README covers the first part of the project: setting up the marker pose publisher and changing the streaming ID, based on handwritten notes and the provided Python script.

---

## Overview

This ROS 2 node subscribes to `/rigid_bodies` and publishes the pose of a specific marker to `/marker_pose`. The marker is identified by its `rigid_body_name` (e.g., `'2'`).

You can change the marker or streaming ID by modifying the value checked in the script or by using launch file parameters.

---

## Marker Pose Publisher

### Functionality

- **Subscribes**: `/rigid_bodies` topic (`mocap4r2_msgs/RigidBodies`)
- **Publishes**: `/marker_pose` topic (`geometry_msgs/PoseStamped`)
- **Filters**: Only publishes pose where `rigid_body_name == '2'` (change `'2'` to your desired marker/ID).

### How to Run

1. **Update the Marker ID**
    - In the script, change:
      ```python
      if rb.rigid_body_name == '2':
      ```
      to your required marker string/number.

2. **Launch the Node**
    - Use your ROS 2 launch system or run directly:
      ```bash
      ros2 run <your_package_name> marker_pose_publisher.py
      ```

3. **Changing Streaming ID**
    - You can parameterize the marker ID using a launch file by adding a parameter and reading it inside the node.
    - Example launch argument:
      ```xml
      <param name="marker_id" value="2"/>
      ```
    - Update the node to use the parameter instead of hardcoded `'2'`.

### Debug Output

The node prints published pose information for debugging:
- Position `(x, y, z)`
- Orientation `(x, y, z, w)`
- Frame ID

---

## SSH & Marker Connector Notes

- **SSH Keygen**: Only set up SSH keys if you need remote access to hardware. For marker pose publishing, this step is typically not required.
- **Marker Connector**: Not needed at this stage—pose publishing works directly from the subscription and publisher.

---

## Quick Start

1. Change marker ID in code or launch.
2. Run the node.
3. Check `/marker_pose` topic for published marker pose.

---

## References

- `geometry_msgs/msg/PoseStamped`
- `mocap4r2_msgs/msg/RigidBodies`
- See `/ERRORS` folder for troubleshooting.

---

**Handwritten Notes:**
- “Marker_pose publisher, pkg”
- “launch file, change streaming id into the number you want”
- “SSH keygen”
- “No need marker connector because the file…”

---

## Full Code Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies

class MarkerPosePublisher(Node):
    def __init__(self):
        super().__init__('marker_pose_publisher')
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_callback,
            10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/marker_pose', 10)

    def rigid_bodies_callback(self, msg):
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == '2':
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose = rb.pose
                self.pose_pub.publish(pose_msg)
                # Print position, orientation, and frame_id for debugging
                self.get_logger().info(
                    f"Published /marker_pose\n"
                    f"  position: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}\n"
                    f"  orientation: x={pose_msg.pose.orientation.x:.3f}, y={pose_msg.pose.orientation.y:.3f}, "
                    f"z={pose_msg.pose.orientation.z:.3f}, w={pose_msg.pose.orientation.w:.3f}\n"
                    f"  frame_id: {pose_msg.header.frame_id}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---
For further setup, hardware integration, or connector usage, see subsequent documentation.
