from move_arm_interfaces.srv import SetPose

import rclpy
from rclpy.node import Node


class SetPoseClient(Node):
    def __init__(self):
        super().__init__('set_pose_client')
        self.client = self.create_client (SetPose, 'set_pose')
        self.get_logger().info("Client Ready  ")

    def send_request(self, pose: str):
        # waitfor the service 
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("'set_pose' service not active.")
            return None

        req = SetPose.Request()
        req.pose = pose

        # call service async
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed!")
            return None


def main():
    rclpy.init()
    node = SetPoseClient()

    try:
        while True:
            cmd = input("Enter pose (home/ready/pick/place): ").strip()
            if not cmd:
                continue

            res = node.send_request(cmd)
            if res:
                print(res.message)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
