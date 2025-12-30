

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/arm_controller/joint_trajectory"
JOINTS = ["joint1", "joint3"] #[base,elbow]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.poses = [
            [0,0],
            [0.8, 0],
            [0.8,-1.0],
            [0, -1.0],
            [-0.8,-1.0],
            [-0.8, 0],
            [-0.8,1.0],
            [0, 1.0],
            [0.8,1.0],
            [0.8,0]
            
            
            
            
        ]
        self.phase = 0
        self.get_logger().info(f"🦾 Started. Publishing to {TRAJ_TOPIC} | Joints: {JOINTS}")
        self.timer = self.create_timer(2, self.tick)

    def tick(self):
        base_pos, elbow_pos = self.poses[self.phase]

        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint(
            positions=[base_pos, elbow_pos],
            time_from_start=Duration(sec=2)
        )
        msg.points.append(point)
        self.pub.publish(msg)

        base_dir = "Left ←" if base_pos > 0 else "Right →" if base_pos < 0 else "Center"
        elbow_dir = "Down ↓" if elbow_pos > 0 else "Up ↑" if elbow_pos < 0 else "Center"


        self.get_logger().info(
            f"[Phase {self.phase+1}/{len(self.poses)}] "
            f"Base: {base_dir} ({base_pos:.2f}) | Elbow: {elbow_dir} ({elbow_pos:.2f})"
        )

        self.phase = (self.phase + 1) % len(self.poses)

def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Ctrl+C — shutting down cleanly.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
