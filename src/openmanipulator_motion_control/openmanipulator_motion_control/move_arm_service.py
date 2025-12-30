import rclpy
from rclpy.node import Node
from move_arm_interfaces.srv import MoveArm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveArmServer(Node):
    def __init__(self):
        super().__init__('move_arm_service')

     
        self.srv = self.create_service(MoveArm, 'move_arm', self.move_arm_callback)

        
        self.publisher = self.create_publisher(JointTrajectory,"/simple_joint_controller/simple_trajectory",10)


        self.get_logger().info("/move_arm service ready (ROS2 Control mode).")

       
        self.targets = {
            "center": [0.0, 0.0, 0.0, 0.0],
            "right": [0.8, 0.0, 0.0, 0.0],
            "left": [-0.8, 0.0, 0.0, 0.0],
            "up": [0.0, 0.0, -1.0, 0.0],
            "top": [0.0, 0.0, -1.0, 0.0],
            "down": [0.0, 0.0, 1.0, 0.0],
            "bottom": [0.0, 0.0, 1.0, 0.0],
        }

    def move_arm_callback(self, request, response):
        direction = request.direction.lower()
        self.get_logger().info(f"Received direction: {direction}")

        if direction not in self.targets:
            response.success = False
            response.message = f"Unknown direction: {direction}"
            return response

        
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        point = JointTrajectoryPoint()
        point.positions = self.targets[direction]
        point.time_from_start.sec = 5  # moving time taken

        msg.points.append(point)

        
        self.publisher.publish(msg)
        self.get_logger().info(f"published trajectory to move {direction}")

        response.success = True
        response.message = f"Moved to {direction}: {self.targets[direction]}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MoveArmServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
