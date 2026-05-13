"""
A ROS2 node that publishes joint states for the H1-2 humanoid robot.

This node publishes to the /joint_states topic at 50 Hz, simulating
joint movements using sinusoidal functions for demonstration purposes.

Launch it as: 
    ros2 launch h1_2_description display_handless.launch.py
    ros2 run h1_2_model joint_publisher
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class JointPublisher(Node):

    def __init__(self):
        super().__init__('h1_2_joint_publisher')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish)      # 50 Hz
        self.t = 0.0

        # Joint names matching URDF
        self.joint_names = ['left_hip_yaw_joint', 'left_hip_pitch_joint', 
                            'left_hip_roll_joint', 'left_knee_joint',
                            'left_ankle_pitch_joint', 'left_ankle_roll_joint',
                            'right_hip_yaw_joint', 'right_hip_pitch_joint',
                            'right_hip_roll_joint', 'right_knee_joint',
                            'right_ankle_pitch_joint', 'right_ankle_roll_joint',
                            'torso_joint',
                            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                            'left_shoulder_yaw_joint', 'left_elbow_joint',
                            'left_wrist_roll_joint', 'left_wrist_pitch_joint',
                            'left_wrist_yaw_joint',
                            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 
                            'right_shoulder_yaw_joint', 'right_elbow_joint',
                            'right_wrist_roll_joint', 'right_wrist_pitch_joint',
                            'right_wrist_yaw_joint'
        ]

        # Broadcaster for world -> pelvis TF
        self.tf_broadcaster = TransformBroadcaster(self)
        # Message template for the pelvis TF
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'world'
        self.tf.child_frame_id = 'pelvis'


    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Example motion: small sinusoid on elbows
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('left_elbow_joint')] = 0.5 * math.sin(self.t)
        positions[self.joint_names.index('right_elbow_joint')] = -0.5 * math.sin(self.t)

        msg.position = positions
        self.pub.publish(msg)

        self.t += 0.02

        # Publish TF world -> pelvis (floating base)
        xb = [0.0, 0.0, 0.9832, 0.0, 0.0, 0.0, 1.0] 
        self.tf.transform.translation.x = xb[0]
        self.tf.transform.translation.y = xb[1]
        self.tf.transform.translation.z = xb[2]
        self.tf.transform.rotation.x = xb[3]
        self.tf.transform.rotation.y = xb[4]
        self.tf.transform.rotation.z = xb[5]
        self.tf.transform.rotation.w = xb[6]

        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf)


def main():
    rclpy.init()
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
