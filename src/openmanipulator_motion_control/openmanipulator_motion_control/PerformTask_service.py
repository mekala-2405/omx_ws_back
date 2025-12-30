#!/usr/bin/env python3
"""
PerformTask_service_fixed.py
ROS 2 Service Node for Pick-and-Place using Trigger service.
"""

import math
import time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import threading


TRAJ_TOPIC = "/simple_joint_controller/simple_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]


def deg2rad(deg_list):
    return [math.radians(d) for d in deg_list]


class PickPlaceService(Node):
    def __init__(self):
        super().__init__("perform_task_service")

        # Publishers and Clients
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Create the Trigger service
        self.srv = self.create_service(Trigger, 'perform_task', self.perform_task_callback)

        self.get_logger().info("Pick & Place service node ready! Call /perform_task to execute.")

    # ---- Helper Functions ----

    def move_arm(self, joint_deg, duration=2.0):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = deg2rad(joint_deg)
        point.time_from_start = Duration(sec=int(duration))
        msg.points.append(point)
        self.pub.publish(msg)
        self.get_logger().info(f"Moving arm → {joint_deg}°")
        time.sleep(duration + 0.5)

    def move_gripper(self, position, effort=2.0, settle_time=3.0):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort

        self.gripper_client.wait_for_server()
        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self.get_logger().info(f"Gripper → {position:.3f} m (effort={effort})")
        time.sleep(settle_time)

    # ---- Pick-and-Place Core Logic ----
    def execute_sequence(self):
        self.get_logger().info("Starting pick-and-place sequence...")

        # 1️⃣ Home position — gripper closed
        self.move_gripper(0.0)
        self.move_arm([0, 0, 0, 0])
        

        # 2️⃣ Open gripper before picking
        self.move_gripper(0.02)

        # 3️⃣ Move near cube
        self.move_arm([0, 40, 0, 0])

        # 4️⃣ Grip cube tightly
        self.move_gripper(0.0, effort=2.0, settle_time=2.0)

        # 5️⃣ Return to home with cube
        self.move_arm([0, 0, 0, 0])

        # 6️⃣ Rotate to left (place area)
        self.move_arm([77, 0, 0, 0])

        # 7️⃣ Bend down to place
        self.move_arm([77, 27, 0, 0])

        # 8️⃣ Release cube
        self.move_gripper(0.02, effort=1.0, settle_time=2.0)

        # 9️⃣ Return home
        self.move_arm([77, 0, 0, 0])
        self.move_arm([0, 0, 0, 0])
        self.move_gripper(0.0)

        self.get_logger().info(" Pick-and-place complete!")

    # ---- Service Callback ----
    def perform_task_callback(self, request, response):
        """Runs pick-and-place asynchronously so service returns immediately."""
        self.get_logger().info("Service called: /perform_task")
        response.success = True
        response.message = "Pick-and-place started. Check logs for progress."

        # Run in background thread so the callback returns immediately
        threading.Thread(target=self.execute_sequence, daemon=True).start()

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down perform_task_service node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
