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

        self.get_logger().info('UR5e Controller Node has been started.')

    def move_robot(self):

        #wait for the controllers action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Contreoller connected')

        #create the goal message

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        #create a trajectory point
        point = JointTrajectoryPoint()

        point.positions = [0.3, -1.57, 0.0, -1.57, 0.0, 0.0]

        # robot should reach te position after 3 seconds
        point.time_from_start.sec = 3

        goal.trajectory.points.append(point)

        # send the goal
        self.get_logger().info('Sending goal...')

        future = self._action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        # wait for the movement to finish
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code == 0:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed with error code: {result.error_code}')

        
def main(args=None):

    rclpy.init(args=args)

    robot = UR5eController()

    robot.move_robot()

    robot.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()