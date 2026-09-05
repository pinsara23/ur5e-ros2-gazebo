import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class UR5eController(Node):

    def __init__(self):
        super().__init__('ur5e_controller')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info(
            'UR5e Controller Node has been started.'
        )

    def create_point(self, positions, seconds):
        """
        Create a trajectory point.

        positions: list of six joint positions in radians
        seconds: time from the beginning of the trajectory
        """

        point = JointTrajectoryPoint()

        point.positions = positions
        point.time_from_start.sec = seconds

        return point

    def move_robot(self):

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Controller connected')

        # Create the trajectory goal
        goal = FollowJointTrajectory.Goal()

        # Define the order of joints used by our trajectory
        goal.trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Create trajectory waypoints
        point1 = self.create_point(
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            2
        )

        point2 = self.create_point(
            [0.5, -1.2, -0.5, -1.3, 0.2, 0.0],
            4
        )

        point3 = self.create_point(
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            6
        )

        # Add waypoints to the trajectory
        goal.trajectory.points = [
            point1,
            point2,
            point3
        ]

        self.get_logger().info(
            'Sending trajectory with 3 waypoints...'
        )

        # Send trajectory to the controller
        future = self._action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Wait for the trajectory result
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result().result

        if result.error_code == 0:
            self.get_logger().info(
                'Trajectory succeeded!'
            )
        else:
            self.get_logger().info(
                f'Trajectory failed with error code: '
                f'{result.error_code}'
            )


def main(args=None):

    rclpy.init(args=args)

    robot = UR5eController()

    robot.move_robot()

    robot.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()