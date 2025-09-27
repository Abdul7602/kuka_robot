#!/usr/bin/env python3
"""
ee_move_cartesian_ros2.py
Move LBR iiwa14 end-effector to a Cartesian target using ROS2 MoveGroup action.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped

class MoveEECartesian(Node):
    def __init__(self):
        super().__init__('endeffector')

        # Connect to MoveGroup action server
        self._client = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        self.get_logger().info("Connected!")

        # Move to Cartesian target
        self.move_to_pose()

    def move_to_pose(self):
        # PoseStamped for EE target
        target_pose = PoseStamped()
        target_pose.header.frame_id = "lbr_link_0"
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.0707
        target_pose.pose.position.z = 0.091
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
<<<<<<< HEAD
        target_pose.pose.orientation.z = -0.9846
=======
        target_pose.pose.orientation.z = 0.9846
>>>>>>> 5088c80 (Add MoveIt scripts and resources)
        target_pose.pose.orientation.w = -0.1748

        # Create a simple goal constraint
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header.frame_id = "lbr_link_0"
        oc.link_name = "lbr_link_ee"
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        # Motion plan request
        request = MotionPlanRequest()
        request.group_name = "arm"
        request.goal_constraints.append(constraints)
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        # Send goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        self.get_logger().info(f"Sending EE Cartesian goal...")
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Goal sent!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveEECartesian()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
<<<<<<< HEAD


#- header:
#    frame_id: lbr_link_6
#  child_frame_id: lbr_link_7
# transform:
#    translation:
#      x: 0.0
#      y: 0.0707
#      z: 0.091
#   rotation:
#    x: 0.0
#      y: 0.0
#   z: 0.9846
#     w: -0.1748
=======
>>>>>>> 5088c80 (Add MoveIt scripts and resources)
