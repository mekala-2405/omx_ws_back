#!/usr/bin/env python3
"""
Simplified feedback-driven pick & place
- Slow during pickup and place
- Normal speed otherwise
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

JOINTS = ["joint1", "joint2", "joint3", "joint4"]
TRAJ_TOPIC = "/simple_joint_controller/simple_trajectory"
ACCEPTED_ERROR = 0.13
STABLE_CYCLES = 3
LOOP_HZ = 20
MAX_WAIT = 8.0

def deg2rad(values):
    return [math.radians(v) for v in values]

class SimplePickPlace(Node):
    def __init__(self):
        super().__init__("perform_task_feedback_simple")

        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.srv = self.create_service(Trigger, 'perform_task_feedback_simple', self.perform_task_cb)

        self.get_logger().info("Simple Pick & Place (slow pickup/place) ready")

    def joint_callback(self, msg):
        self.joint_state = msg

    def get_current_positions(self):
        if not self.joint_state:
            return None
        current = []
        for j in JOINTS:
            if j in self.joint_state.name:
                idx = self.joint_state.name.index(j)
                current.append(self.joint_state.position[idx])
        return current

    def move_arm(self, target_deg, slow=False):
        target = deg2rad(target_deg)
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = target

        duration = 6 if slow else 3  # slower for pick/place
        pt.time_from_start = Duration(sec=duration)

        msg.points.append(pt)
        self.pub.publish(msg)
        self.get_logger().info(f"Moving arm to {target_deg}° (duration {duration}s)")

        start = time.time()
        stable = 0
        while time.time() - start < MAX_WAIT:
            current = self.get_current_positions()
            if not current:
                time.sleep(0.1)
                continue

            errors = [abs(a - b) for a, b in zip(current, target)]
            self.get_logger().info(f"Errors: {[round(e,3) for e in errors]}")

            if all(e < ACCEPTED_ERROR for e in errors):
                stable += 1
                if stable >= STABLE_CYCLES:
                    self.get_logger().info("Arm converged")
                    return True
            else:
                stable = 0
            time.sleep(1.0 / LOOP_HZ)

        self.get_logger().warn("Arm timeout → going HOME")
        self.go_home()
        return False

    def move_gripper(self, pos, effort=2.0):
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Gripper set to {pos}")
        time.sleep(1.0)
        return True

    def go_home(self):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0, 0.0, 0.0]
        pt.time_from_start = Duration(sec=3)
        msg.points.append(pt)
        self.pub.publish(msg)
        self.get_logger().info("Moving HOME")

    def execute_sequence(self):
        steps = [
            ("grip", 0.02),                 # open
            ("pose", [0, 40, 0, 0], True),  # move to pick (slow)
            ("grip", 0.0),                  # close grip
            ("pose", [0, 0, 0, 0], False),  # lift/home
            ("pose", [77, 27, 0, 0], True), # move to place (slow)
            ("grip", 0.02),                 # release cube
            ("pose", [77, 10, 0, 0], True), # small lift after placing (safe)
            ("pose", [0, 0, 0, 0], False),  # return home
        ]

        for i, step in enumerate(steps, 1):
            t = step[0]
            val = step[1]
            slow = step[2] if len(step) > 2 else False
            self.get_logger().info(f"[{i}/{len(steps)}] {t}={val} slow={slow}")
            if t == "pose":
                ok = self.move_arm(val, slow)
            else:
                ok = self.move_gripper(val)
            if not ok:
                break

        self.get_logger().info("Task finished")

    def perform_task_cb(self, request, response):
        response.success = True
        response.message = "Pick & place started"
        threading.Thread(target=self.execute_sequence, daemon=True).start()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimplePickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"Shutdown warning (safe to ignore): {e}")


if __name__ == "__main__":
    main()
