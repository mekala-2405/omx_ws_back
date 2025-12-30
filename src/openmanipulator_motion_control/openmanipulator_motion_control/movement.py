import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/simple_joint_controller/simple_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        # Fixed poses (4 joints each)
        self.poses = [
            [0, 0, 0, 0],        # Center
            [0.8, 0, 0, 0],      # Right
            [-0.8, 0, 0, 0],     # Left
                 
            [0, 0, -1.0, 0],     # Up
            [0, 0, 1.0, 0],      # Down
            [0, 0, 0, 0]         # Center again
        ]

        self.phase = 0
        self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC} | Joints: {JOINTS}")
        self.timer = self.create_timer(3.0, self.tick)

    def tick(self):
        pos = self.poses[self.phase]

        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint(
            positions=pos,
            time_from_start=Duration(sec=2, nanosec=0)
        )
        msg.points.append(point)
        self.pub.publish(msg)

        # Just for logging directions
        base_dir = "Left ←" if pos[0] > 0 else "Right →" if pos[0] < 0 else "Center"
        elbow_dir = "Down ↓" if pos[1] > 0 else "Up ↑" if pos[1] < 0 else "Center"

        self.get_logger().info(
            f"[Phase {self.phase+1}/{len(self.poses)}] "
            f"Base: {base_dir} ({pos[0]:.2f}) | Elbow: {elbow_dir} ({pos[1]:.2f})"
        )

        self.phase = (self.phase + 1) % len(self.poses)

def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C — shutting down cleanly.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
