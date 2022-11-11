import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import JointState


class DummyJointPublisher(Node):

    def __init__(self):
        super().__init__('dummy_joint_publisher')
        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Create a JointState message
        self.joint_state = JointState()

        # self.joint_state.name = [  # Omniverse tutorial demo: "7. ROS2 Joint Control: Extension Python Scripting"
        #     "panda_joint1",
        #     "panda_joint2",
        #     "panda_joint3",
        #     "panda_joint4",
        #     "panda_joint5",
        #     "panda_joint6",
        #     "panda_joint7",
        #     "panda_finger_joint1",
        #     "panda_finger_joint2",
        # ]

        self.joint_state.name = [  # Heggem, Whal robot
            'joint_a1',
            'joint_a2',
            'joint_a3',
            'joint_a4',
            'joint_a5',
            'joint_a6',
            'joint_a7',
        ]

        num_joints = len(self.joint_state.name)

        # make sure kit's editor is playing for receiving messages
        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        # self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]  # Isaac sim ros demo
        self.default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Heggem, Whal robot

        # limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5

        # position control the robot to wiggle around each joint
        self.time_start = time.time()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_position = (
            np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
        )
        self.joint_state.position = joint_position.tolist()

        # Publish the message to the topic
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)

    dummy_joint_publisher = DummyJointPublisher()

    rclpy.spin(dummy_joint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dummy_joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()