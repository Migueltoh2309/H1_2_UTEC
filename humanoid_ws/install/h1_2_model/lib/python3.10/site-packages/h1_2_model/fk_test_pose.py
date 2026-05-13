"""
Forward Kinematics Test Node (position and orientation) for H1-2 Robot

This node computes the forward kinematics of the H1-2 robot using Pinocchio
and publishes the end-effector pose as a TF in RViz for verification.
It also publishes the joint states for visualization and broadcasts the TF
from the world frame to the pelvis frame ("floating base").

Run it as: ros2 launch h1_2_model display_handless_fk.launch.py
           ros2 run h1_2_model fk_test_pose
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import numpy as np
import pinocchio as pin
import math
import os
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FKTestNode(Node):

    def __init__(self):
        super().__init__('h1_2_fk_test')

        # -------------------------------
        # Load URDF into Pinocchio
        # -------------------------------
        pkg_share = get_package_share_directory('h1_2_description')
        urdf_path = os.path.join(pkg_share, 'urdf', 'h1_2_handless.urdf')

        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        self.ee_frame = 'left_wrist_yaw_link'
        self.ee_frame_id = self.model.getFrameId(self.ee_frame)

        # -------------------------------
        # Joint names
        # -------------------------------
        self.joint_names = [
            'left_hip_yaw_joint', 'left_hip_pitch_joint', 'left_hip_roll_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_yaw_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'torso_joint',
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint', 'left_elbow_joint',
            'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint', 'right_elbow_joint',
            'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
        ]

        self.nj = len(self.joint_names)

        # -------------------------------
        # Joint States
        # -------------------------------
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.t = 0.0

        # JointState message template
        self.js = JointState()
        self.js.name = self.joint_names

        # -----------------------------------
        # Broadcaster
        # -----------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)
        # TF message world -> pelvis
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'world'
        self.tf.child_frame_id = 'pelvis'
        # TF message world -> left_wrist_yaw_link_fk
        self.ee_tf = TransformStamped()
        self.ee_tf.header.frame_id = 'world'
        self.ee_tf.child_frame_id = 'fk_left_wrist_yaw_link'


    def publish_floating_base_tf(self, q):
        # ----------------------------------
        # Publish TF world -> pelvis
        # ----------------------------------
        self.tf.transform.translation.x = float(q[0])
        self.tf.transform.translation.y = float(q[1])
        self.tf.transform.translation.z = float(q[2])
        self.tf.transform.rotation.x = float(q[3])
        self.tf.transform.rotation.y = float(q[4])
        self.tf.transform.rotation.z = float(q[5])
        self.tf.transform.rotation.w = float(q[6])

        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf)


    def pinocchio_se3_to_transform(self, M, tf_msg):
        # Position
        tf_msg.transform.translation.x = float(M.translation[0])
        tf_msg.transform.translation.y = float(M.translation[1])
        tf_msg.transform.translation.z = float(M.translation[2])

        # Rotation matrix to quaternion
        q = pin.Quaternion(M.rotation)
        q.normalize()

        tf_msg.transform.rotation.x = float(q.x)
        tf_msg.transform.rotation.y = float(q.y)
        tf_msg.transform.rotation.z = float(q.z)
        tf_msg.transform.rotation.w = float(q.w)


    def update(self):
        # ----------------------------------
        # Configuration vector q
        # ----------------------------------
        q = np.zeros(self.model.nq)

        # Floating base (pelvis)
        q[0:3] = np.array([0.0, 0.0, 0.9832])         # Position
        q[3:7] = np.array([0.0, 0.0, 0.0, 1.0])       # Orientation (quaternion:, x,y,z,w   )

        # Actuated joints
        q[7 + self.joint_names.index('left_shoulder_roll_joint')] = 0.5
        q[7 + self.joint_names.index('left_shoulder_pitch_joint')] = -0.5
        q[7 + self.joint_names.index('left_elbow_joint')] = 0.5 * math.sin(self.t)

        # ----------------------------------
        # Publish JointState (for RViz robot)
        # ----------------------------------
        self.js.header.stamp = self.get_clock().now().to_msg()
        self.js.position = q[7:].tolist()
        # Publish actuated joints
        self.joint_pub.publish(self.js)
        # Publish floating base TF
        self.publish_floating_base_tf(q[0:7])

        # ----------------------------------
        # Forward kinematics
        # ----------------------------------
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        ee_pose = self.data.oMf[self.ee_frame_id]

        self.ee_tf.header.stamp = self.get_clock().now().to_msg()
        self.pinocchio_se3_to_transform(ee_pose, self.ee_tf)
        # Show a TF (frame) with the computed pose
        self.tf_broadcaster.sendTransform(self.ee_tf)

        self.t += 0.05

def main():
    rclpy.init()
    node = FKTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
