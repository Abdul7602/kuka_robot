#!/usr/bin/env python3
"""
lbr_move_points_joint_probe_scan.py
Move LBR iiwa14 end-effector to fixed position, rotate last joint 180°, then simulate
ultrasound scanning by oscillating last joint and slightly moving the 4th joint simultaneously.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
import math
import time

# Precomputed IK solution for Point A
POINT_A_JOINTS = [
    math.radians(-66),
    math.radians(32),
    math.radians(68),
    math.radians(-57),
    math.radians(-39),
    math.radians(-74),
    math.radians(-160)
]

JOINT_NAMES = ["lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7"]

class MoveEE(Node):
    def __init__(self):
        super().__init__('lbr_move_points_joint_probe_scan')

        # Connect to MoveGroup action server
        self._client = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        self.get_logger().info("Connected!")

        # Step 1: Move to Point A
        self.get_logger().info("Moving to Point A (IK solution)...")
        self.move_to_joint_goal(POINT_A_JOINTS)
        time.sleep(1.0)

        # Step 2: Rotate last joint 180 degrees
        joints_rotated = POINT_A_JOINTS.copy()
        joints_rotated[-1] += math.radians(180)
        self.get_logger().info("Rotating last joint 180°...")
        self.move_to_joint_goal(joints_rotated)
        time.sleep(1.0)

        # Step 3: Oscillate last joint and slightly move 4th joint for scanning
        self.get_logger().info("Starting scanning motion (last joint oscillation + 4th joint tilt)...")
        last_joint_amplitude = math.radians(10)  # 10 degrees up/down
        fourth_joint_amplitude = math.radians(5)  # small tilt
        current_joints = joints_rotated.copy()

        for i in range(3):  # 3 oscillations
            # Move up
            current_joints[-1] += last_joint_amplitude
            current_joints[3] += fourth_joint_amplitude
            self.get_logger().info(f"Scan {i+1} - up")
            self.move_to_joint_goal(current_joints)
            time.sleep(0.5)

            # Move down
            current_joints[-1] -= 2*last_joint_amplitude
            current_joints[3] -= 2*fourth_joint_amplitude
            self.get_logger().info(f"Scan {i+1} - down")
            self.move_to_joint_goal(current_joints)
            time.sleep(0.5)

            # Return to center
            current_joints[-1] += last_joint_amplitude
            current_joints[3] += fourth_joint_amplitude

        self.get_logger().info("Scanning motion complete!")

    def move_to_joint_goal(self, joint_positions):
        # Build joint constraints
        constraints = Constraints()
        for name, angle in zip(JOINT_NAMES, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        # Motion plan request
        request = MotionPlanRequest()
        request.group_name = "arm"
        request.goal_constraints.append(constraints)
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        # Build MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request

        self.get_logger().info(
            f"Sending joint goal: {[round(math.degrees(j), 2) for j in joint_positions]} degrees"
        )
        future_goal = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future_goal)
        time.sleep(0.2)  # small pause for smooth motion

def main(args=None):
    rclpy.init(args=args)
    node = MoveEE()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
