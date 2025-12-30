#!/usr/bin/env python3
"""
Pick & Place Action Server with Preemption Support
- Action-based interface with goal/feedback/cancel
- Graceful unwind to SAFE pose on cancel/abort
- Comprehensive logging system
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration


from move_arm_interfaces.action import PickPlace

JOINTS = ["joint1", "joint2", "joint3", "joint4"]
TRAJ_TOPIC = "/simple_joint_controller/simple_trajectory"
ACCEPTED_ERROR = 0.13
STABLE_CYCLES = 3
LOOP_HZ = 20
MAX_WAIT = 8.0
SAFE_POSE = [0, 0, 0, 0]  # Home position as safe pose

def deg2rad(values):
    return [math.radians(v) for v in values]

class PickPlaceActionServer(Node):
    def __init__(self):
        super().__init__("pickplace_action_server")
        
        # Callback group for concurrent execution
        self.cb_group = ReentrantCallbackGroup() #research about this more 
        
        # Publishers and subscribers
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_controller/gripper_cmd',
            callback_group=self.cb_group
        )
        self.joint_state = None
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10,
            callback_group=self.cb_group
        )

        self._action_server = ActionServer(
            self,
            PickPlace,
            'pickplace_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        
        # State tracking
        self._goal_handle = None
        self._goal_lock = threading.Lock() #ante oka task iyye time ki no other tasks??
        self._cancel_requested = False
        
        
        self.get_logger().info("Pick & Place Action Server READY")
        self.get_logger().info("Action: /pickplace_action")
        
        

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

    def goal_callback(self, goal_request):
        
        self.get_logger().info("New goal request received")
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn(" Goal already active - rejecting new goal") # correct 
                return GoalResponse.REJECT
        self.get_logger().info(" Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation requests"""
        self.get_logger().warn("ANCEL REQUEST received")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def check_cancel(self):
        """Check if cancellation was requested"""
        return self._cancel_requested

    def move_arm(self, target_deg, slow=False, step_name=""):
        """Move arm with cancellation checking"""
        if self.check_cancel():
            self.get_logger().warn(f"Cancelled during: {step_name}")
            return False
            
        target = deg2rad(target_deg)
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = target

        duration = 6 if slow else 3
        pt.time_from_start = Duration(sec=duration)

        msg.points.append(pt)
        self.pub.publish(msg)
        self.get_logger().info(f" Moving arm: {target_deg}° (duration={duration}s) [{step_name}]")

        start = time.time()
        stable = 0
        while time.time() - start < MAX_WAIT:
            # Check for cancellation during movement
            if self.check_cancel():
                self.get_logger().warn(f" Cancelled mid-movement: {step_name}")
                return False
                
            current = self.get_current_positions()
            if not current:
                time.sleep(0.1)
                continue

            errors = [abs(a - b) for a, b in zip(current, target)]
            max_error = max(errors)
            
            # log 0.5 sec
            if int((time.time() - start) * 2) % 1 == 0:
                self.get_logger().info(f"   Error: {[round(e,3) for e in errors]} (max={max_error:.3f})")

            if all(e < ACCEPTED_ERROR for e in errors):
                stable += 1
                if stable >= STABLE_CYCLES:
                    self.get_logger().info(f"Arm converged to target [{step_name}]")
                    return True
            else:
                stable = 0
            time.sleep(1.0 / LOOP_HZ)

        self.get_logger().error(f"  Timeout waiting for convergence [{step_name}]")
        return False

    def move_gripper(self, pos, effort=2.0, step_name=""):
        
        if self.check_cancel():
            self.get_logger().warn(f" Cancelled during: {step_name}")
            return False
            
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = effort
        
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("  Gripper action server not available")
            return False
            
        future = self.gripper_client.send_goal_async(goal)
        
        # Wait with cancellation checking
        start_time = time.time()
        while not future.done():
            if self.check_cancel():
                self.get_logger().warn(f" Cancelled during gripper move: {step_name}")
                # Try to cancel the gripper action too
                return False
            if time.time() - start_time > 5.0:
                self.get_logger().error("  Gripper timeout")
                return False
            time.sleep(0.1)
            
        self.get_logger().info(f" Gripper: pos={pos} [{step_name}]")
        time.sleep(1.0)
        return True

    def go_safe_pose(self):
        
        self.get_logger().warn(" Moving to SAFE POSE (home)")
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in SAFE_POSE]
        pt.time_from_start = Duration(sec=3)
        msg.points.append(pt)
        self.pub.publish(msg)
        time.sleep(3.5)  # Wait for movement to complete
        self.get_logger().info(" Safe pose reached")

    def execute_callback(self, goal_handle):
       
        
        self.get_logger().info(" STARTING PICK & PLACE SEQUENCE")
        
        
        with self._goal_lock:
            self._goal_handle = goal_handle
            self._cancel_requested = False

        # Define task sequence
        steps = [
            ("grip", 0.02, False, "Open gripper"),
            ("pose", [0, 40, 0, 0], True, "Move to pick position"),
            ("grip", 0.0, False, "Close gripper (grab)"),
            ("pose", [0, 0, 0, 0], False, "Lift object (home)"),
            ("pose", [77, 27, 0, 0], True, "Move to place position"),
            ("grip", 0.02, False, "Open gripper (release)"),
            ("pose", [77, 10, 0, 0], True, "Lift after placing"),
            ("pose", [0, 0, 0, 0], False, "Return home"),
        ]

        total_steps = len(steps)
        feedback_msg = PickPlace.Feedback()
        result = PickPlace.Result()
        
        for i, step in enumerate(steps, 1):
            step_type = step[0]
            value = step[1]
            slow = step[2]
            description = step[3]
            
            # Publish feedback
            feedback_msg.current_step = i
            feedback_msg.total_steps = total_steps
            feedback_msg.step_description = description
            feedback_msg.progress_percentage = (i / total_steps) * 100.0
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f" Step {i}/{total_steps}: {description}")
            
            # Execute step
            if step_type == "pose":
                ok = self.move_arm(value, slow, description)
            else:
                ok = self.move_gripper(value, step_name=description)
            
            # Handle failure or cancellation
            if not ok:
                if self.check_cancel():
                    
                    self.get_logger().warn(" TASK CANCELLED - Initiating safe abort")
                    
                    
                    # Open gripper if holding object
                    if i >= 3 and i <= 6:  # Between grab and release
                        self.get_logger().info("Opening gripper (safety)")
                        self.move_gripper(0.02, step_name="Emergency release")
                    
                    # Move to safe pose
                    self.go_safe_pose()
                    
                    result.success = False
                    result.message = f"Cancelled at step {i}: {description}"
                    result.steps_completed = i - 1
                    goal_handle.canceled()
                    
                    self.get_logger().warn(" Safe abort complete")
                    with self._goal_lock:
                        self._goal_handle = None
                    return result
                else:
                    self.get_logger().error(f"Step {i} FAILED: {description}")
                    self.go_safe_pose()
                    
                    result.success = False
                    result.message = f"Failed at step {i}: {description}"
                    result.steps_completed = i - 1
                    goal_handle.abort()
                    
                    with self._goal_lock:
                        self._goal_handle = None
                    return result
            
            self.get_logger().info(f" Step {i} complete")

        # Success!
        
        self.get_logger().info("TASK COMPLETED SUCCESSFULLY")
        
        
        result.success = True
        result.message = "Pick and place completed successfully"
        result.steps_completed = total_steps
        goal_handle.succeed()
        
        with self._goal_lock:
            self._goal_handle = None
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    node = PickPlaceActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Spinning node...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(" Keyboard interrupt - shutting down...")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"Shutdown warning (safe to ignore): {e}")


if __name__ == "__main__":
    main()