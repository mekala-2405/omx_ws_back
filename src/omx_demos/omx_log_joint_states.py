#!/usr/bin/env python3
import rclpy, csv, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

LOG_PATH = "/home/ubuntu/logs/omx_joint_log.csv"

class JointLogger(Node):
    def __init__(self):
        super().__init__("joint_logger")
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, qos_profile_sensor_data)
        self.f = open(LOG_PATH, "w", newline="")
        self.w = csv.writer(self.f)
        self.header_written = False
        self.t0 = time.time()
        self.get_logger().info(f"Logging /joint_states to {LOG_PATH}")

    def cb(self, msg: JointState):
        if not self.header_written:
            self.w.writerow(["t"]+[f"{n}_pos" for n in msg.name]+[f"{n}_vel" for n in msg.name]+[f"{n}_eff" for n in msg.name])
            self.header_written = True
        t = time.time() - self.t0
        self.w.writerow([t]+list(msg.position)+list(msg.velocity)+list(msg.effort))

    def destroy_node(self):
        self.f.close(); super().destroy_node()

def main():
        rclpy.init()
        n = JointLogger()
        try:
            rclpy.spin(n)
        except KeyboardInterrupt:
            print("Ctrl+C — stopping logger.")
        finally:
            try:
                n.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == "__main__": main()
