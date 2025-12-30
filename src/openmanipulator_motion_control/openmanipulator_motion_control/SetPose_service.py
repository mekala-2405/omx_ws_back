import rclpy
from rclpy.node import Node
from move_arm_interfaces.srv import SetPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SetPoseService(Node):
    def __init__(self):
        super().__init__('set_pose_service')

        # Create service
        self.srv = self.create_service(SetPose, 'set_pose', self.move_arm_callback)

        # Create ROS2 Control JointTrajectory publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/simple_joint_controller/simple_trajectory",
            10
        )

        self.get_logger().info("/set_pose service ready (ROS2 Control mode).")

        # Predefined poses
        self.targets = {
            "home": [0.0, 0.0, 0.0, 0.0],
            "ready": [0.0, -0.279, -0.401, 0.454],

            # Multi-step waypoints
            "pick": [
                [0.0, 0.017, 0.227, 1.222],
                [0.0, -0.279, -0.332, -0.524]
            ],
            "place": [
                [0.0, -0.279, -0.332, -0.524],
                [0.5, 0.017, 0.227, 0.722]
            ],
        }

    def move_arm_callback(self, request, response):
        pose = request.pose.lower()
        self.get_logger().info(f"Received pose: {pose}")

        if pose not in self.targets:
            response.success = False
            response.message = f"Unknown pose: {pose}"
            return response

        # Extract waypoints
        curr_pos = self.targets[pose]

        # Convert single pose -> list of waypoints
        if isinstance(curr_pos[0], float):
            curr_pos = [curr_pos]

        # Build trajectory message
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        t = 2  # seconds between points
        time_step = 0

        for p in curr_pos:
            point = JointTrajectoryPoint()
            point.positions = p
            time_step += t
            point.time_from_start.sec = time_step
            msg.points.append(point)

        # Publish once
        self.publisher.publish(msg)
        self.get_logger().info(f"Published trajectory for pose: {pose}")

        response.success = True
        response.message = f"Moved to {pose}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetPoseService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
