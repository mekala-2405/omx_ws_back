import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

TRAJ_TOPIC = "/simple_joint_controller/simple_trajectory"
JOINTS = ["joint1", "joint2", "joint3", "joint4"]

def deg_to_rad_list(values_deg):
    """Convert a list of joint angles in degrees → radians."""
    return [math.radians(v) for v in values_deg]

class Demo(Node):
    def __init__(self):
        super().__init__("omx_traj_demo")
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        # ✅ Define poses in DEGREES instead of radians
        self.poses_deg = [
                     
            [0, 0, 0, 0],# default
            [0.0, -0.279, -0.401, 0.454],# ready
            [0.0, 0.017, 0.227, 1.222],#pick
            [0.0, -0.279, -0.332, -0.524],
            [0.0, 0.017, 0.227, 1.222],#place
    
            
            
        ]

        # ✅ Convert them to radians once
        self.poses = [deg_to_rad_list(p) for p in self.poses_deg]

        self.phase = 0
        self.get_logger().info(f"Started. Publishing to {TRAJ_TOPIC} | Joints: {JOINTS}")
        self.timer = self.create_timer(3.0, self.tick)

    def tick(self):
        pos = self.poses[self.phase]
        pos_deg = self.poses_deg[self.phase]

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        point = JointTrajectoryPoint(
            positions=pos,
            time_from_start=Duration(sec=4, nanosec=0)
        )

        msg.points.append(point)
        self.pub.publish(msg)

        # Logging informative output
        self.get_logger().info(
            f"[Phase {self.phase+1}/{len(self.poses)}] "
            f"Degrees: {pos_deg} → Radians: {[round(p, 3) for p in pos]}"
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
