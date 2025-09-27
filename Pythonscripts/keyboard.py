#!/usr/bin/env python3
"""
lbr_move_points_constraints.py
Move LBR iiwa14 end-effector from Point A to Point B using MoveIt2 constraints.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import time


class MoveEE(Node):
    def __init__(self):
        super().__init__('lbr_move_points_constraints')

        # Connect to MoveGroup action server
        self._client = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        self.get_logger().info("Connected!")

        # Define start and end points
        point_a = PoseStamped()
        point_a.header.frame_id = "lbr_link_0"
        point_a.pose.position.x = 0.4
        point_a.pose.position.y = 0.0
        point_a.pose.position.z = 0.4
        point_a.pose.orientation.w = 1.0

        point_b = PoseStamped()
        point_b.header.frame_id = "lbr_link_0"
        point_b.pose.position.x = 0.0
        point_b.pose.position.y = 0.5
        point_b.pose.position.z = 0.4
        point_b.pose.orientation.w = 1.0

        # Move sequence
        for pose, desc in zip([point_a, point_b], ["Point A", "Point B"]):
            self.get_logger().info(f"Moving to {desc}...")
            self.move_to_pose(pose)
            time.sleep(1.0)  # optional pause

        self.get_logger().info("Motion complete!")

    def pose_to_constraints(self, pose_stamped: PoseStamped, link_name="lbr_link_ee"):
        """
        Convert PoseStamped to MoveIt Constraints for MoveGroup goal.
        """
        constraints = Constraints()
        constraints.name = "ee_pose_constraint"

        # Orientation constraint
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = pose_stamped.header.frame_id
        o_constraint.link_name = link_name
        o_constraint.orientation = pose_stamped.pose.orientation
        o_constraint.absolute_x_axis_tolerance = 0.01
        o_constraint.absolute_y_axis_tolerance = 0.01
        o_constraint.absolute_z_axis_tolerance = 0.01
        o_constraint.weight = 1.0

        # Position constraint
        p_constraint = PositionConstraint()
        p_constraint.header.frame_id = pose_stamped.header.frame_id
        p_constraint.link_name = link_name
        p_constraint.target_point_offset.x = 0.0
        p_constraint.target_point_offset.y = 0.0
        p_constraint.target_point_offset.z = 0.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]  # small box around target
        p_constraint.constraint_region.primitives.append(box)
        p_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        p_constraint.weight = 1.0

        constraints.orientation_constraints.append(o_constraint)
        constraints.position_constraints.append(p_constraint)

        return constraints

    def move_to_pose(self, pose_stamped: PoseStamped):
        # Convert pose to goal constraints
        constraints = self.pose_to_constraints(pose_stamped)

        # Motion plan request
        request = MotionPlanRequest()
        request.group_name = "arm"  # change if your MoveIt group is different
        request.goal_constraints.append(constraints)
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        # Build goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request

        # Send goal
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    node = MoveEE()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
