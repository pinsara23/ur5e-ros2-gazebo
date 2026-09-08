import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointStateListener(Node):

    def __init__(self):
        super().__init__('joint_state_listener')

        self.target_reached = False

        self.desired_positions = {
            'shoulder_pan_joint': 0.5,
            'shoulder_lift_joint': -1.57,
            'elbow_joint': 0.0,
            'wrist_1_joint': -1.57,
            'wrist_2_joint': 0.0,
            'wrist_3_joint': 0.0
        }

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self._action_client.wait_for_server()


    def joint_state_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))

        tolerance = 0.01
        all_reached = True

        for joint_name, desired_position in self.desired_positions.items():

            actual_position = joint_positions[joint_name]

            error = desired_position - actual_position

            if abs(error) > tolerance:
                all_reached = False

                print(
                    joint_name,
                    "Actual:", actual_position,
                    "Desired:", desired_position,
                    "Error:", error
                )

        if all_reached and not self.target_reached:
            print("All joints reached target!")
            self.target_reached = True   

    def move_robot(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            self.desired_positions['shoulder_pan_joint'],
            self.desired_positions['shoulder_lift_joint'],
            self.desired_positions['elbow_joint'],
            self.desired_positions['wrist_1_joint'],
            self.desired_positions['wrist_2_joint'],
            self.desired_positions['wrist_3_joint']
        ]

        point.time_from_start.sec = 3

        goal.trajectory.points.append(point)

        self._action_client.send_goal_async(goal)
    

def main(args=None):
    rclpy.init(args=args)

    node = JointStateListener()

    node.move_robot()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()