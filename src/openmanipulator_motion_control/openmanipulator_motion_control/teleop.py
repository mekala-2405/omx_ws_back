import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/simple_joint_controller/simple_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

class ArmTeleop(Node):
    def __init__(self):
        super().__init__("arm_teleop")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        self.pos = {
            "top": [0, 0, -1.0, 0],
            "down": [0, 0, 1.0, 0],
            "left": [0.8, 0, 0, 0],
            "right": [-0.8, 0, 0, 0],
            "center": [0.0, 0.0, 0.0, 0.0]
        }

        self.get_logger().info("Teleop node ready!")
        self.get_logger().info("Commands: top | down | left | right | center | quit")

        self.run_teleop()

    def run_teleop(self):
        while rclpy.ok():
            try:
                cmd = input("Enter direction: ").strip().lower()
                if cmd == "quit":
                    self.get_logger().info("Exiting teleop.")
                    break

                if cmd in self.pos:
                    self.pub_traj(cmd)
                else:
                    self.get_logger().warn(f"Unknown command: {cmd}")

            except KeyboardInterrupt:
                self.get_logger().info("interrupted — shutting down.")
                break

    def pub_traj(self, cmd):
        pos = self.pos[cmd]

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        point = JointTrajectoryPoint(
            positions=pos,
            time_from_start=Duration(sec=2, nanosec=0)
        )

        msg.points.append(point)
        self.pub.publish(msg)

        self.get_logger().info(f"Moved {cmd.upper()} | Positions: {pos}")

def main():
    rclpy.init()
    node = ArmTeleop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
