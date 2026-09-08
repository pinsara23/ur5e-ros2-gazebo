import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateListener(Node):

    def __init__(self):
        super().__init__('joint_state_listener')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))

        desired_position = 0.0
        actual_position = joint_positions["shoulder_pan_joint"]

        error = desired_position - actual_position

        tolerance = 0.01

        if abs(error) <= tolerance:
            print("Target reached!")
        else:
            print("Moving toward target...")    

def main(args=None):

    rclpy.init(args=args)

    node = JointStateListener()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()