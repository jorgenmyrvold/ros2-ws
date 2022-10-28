import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import JointState
import sys, select, termios, tty


settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from keyboard to jogg robot

Joint   1   2   3   4   5   6   7   f1  f2
------------------------------------------
+       q   w   e   r   t   y   u   i   o (open)
-       a   s   d   f   g   h   j   k   l (close)
"""

v = 0.1  # Velocity
key_bindings= {
    'q': np.array([v, 0, 0, 0, 0, 0, 0, 0, 0]),
    'w': np.array([0, v, 0, 0, 0, 0, 0, 0, 0]),
    'e': np.array([0, 0, v, 0, 0, 0, 0, 0, 0]),
    'r': np.array([0, 0, 0, v, 0, 0, 0, 0, 0]),
    't': np.array([0, 0, 0, 0, v, 0, 0, 0, 0]),
    'y': np.array([0, 0, 0, 0, 0, v, 0, 0, 0]),
    'u': np.array([0, 0, 0, 0, 0, 0, v, 0, 0]),
    'i': np.array([0, 0, 0, 0, 0, 0, 0, v*5, 0]),
    'o': np.array([0, 0, 0, 0, 0, 0, 0, 0, v*5]),
    'a': np.array([-v, 0, 0, 0, 0, 0, 0, 0, 0]),
    's': np.array([0, -v, 0, 0, 0, 0, 0, 0, 0]),
    'd': np.array([0, 0, -v, 0, 0, 0, 0, 0, 0]),
    'f': np.array([0, 0, 0, -v, 0, 0, 0, 0, 0]),
    'g': np.array([0, 0, 0, 0, -v, 0, 0, 0, 0]),
    'h': np.array([0, 0, 0, 0, 0, -v, 0, 0, 0]),
    'j': np.array([0, 0, 0, 0, 0, 0, -v, 0, 0]),
    'k': np.array([0, 0, 0, 0, 0, 0, 0, -v*5, 0]),
    'l': np.array([0, 0, 0, 0, 0, 0, 0, 0, -v*5]),
}

class JointJogger(Node):

    def __init__(self):
        print(msg)
        super().__init__('joint_jogger')
        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Create a JointState message
        self.joint_state = JointState()

        self.joint_state.name = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]

        num_joints = len(self.joint_state.name)

        # make sure kit's editor is playing for receiving messages
        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]
        self.last_joint_pos = np.array([0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4])
        # limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5

        # position control the robot to wiggle around each joint
        self.time_start = time.time()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        key = self.get_key()
        if key in key_bindings:
            joint_position = self.last_joint_pos + key_bindings[key]
            if (joint_position <= self.max_joints).all() and (joint_position >= self.min_joints).all():
                self.last_joint_pos = joint_position
        if key == "c":
            print("Exit application")
            exit()
        # try:
        #     
        #     joint_position = self.last_joint_pos + key_bindings[key]
        #     if (joint_position <= self.max_joints).all() and (joint_position >= self.min_joints).all():
        #         self.last_joint_pos = joint_position
        # except:
        #     print(f"{key} is not a valid command")

        self.joint_state.position = self.last_joint_pos.tolist()

        # Publish the message to the topic
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)

    joint_jogger = JointJogger()

    rclpy.spin(joint_jogger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_jogger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()