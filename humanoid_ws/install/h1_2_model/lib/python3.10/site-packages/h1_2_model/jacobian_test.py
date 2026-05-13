"""
Jacobian Test Node for the H1-2 Robot

This node computes the Jacobian of the H1-2 robot using Pinocchio
and visualizes the direction of the end-effector velocity induced
by a small velocity in a selected joint. The direction is visualized
as an arrow in RViz.

Run it as: ros2 launch h1_2_model display_handless_jacobian.launch.py
           ros2 run h1_2_model jacobian_test
"""

import rclpy
from rclpy.node import Node

import math
import numpy as np
import os

import pinocchio as pin

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster

from ament_index_python.packages import get_package_share_directory


def quat_from_yaw(yaw):
    return np.array([ 0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)])


class JacobianTestNode(Node):

    def __init__(self):
        super().__init__('h1_2_jacobian_test')

        # -------------------------------
        # Load URDF into Pinocchio
        # -------------------------------
        pkg_share = get_package_share_directory('h1_2_description')
        urdf_path = os.path.join(pkg_share, 'urdf', 'h1_2_handless.urdf')

        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        # End-effector frame
        self.ee_frame = 'left_wrist_yaw_link'
        self.ee_frame_id = self.model.getFrameId(self.ee_frame)

        # -------------------------------
        # Joint names (actuated only)
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

        # Joint to test
        # ----------------------------------
        self.test_joint_name = 'left_elbow_joint'
        self.test_joint_index = self.joint_names.index(self.test_joint_name)

        # -------------------------------
        # Publishers
        # -------------------------------
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/jacobian_arrow', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.t = 0.0

        # JointState message
        self.js = JointState()
        self.js.name = self.joint_names

        # Jacobian arrow marker
        self.marker = Marker()
        self.marker.header.frame_id = 'world'
        self.marker.ns = 'jacobian'
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD

        self.marker.scale.x = 0.02   # shaft diameter
        self.marker.scale.y = 0.04   # head diameter
        self.marker.scale.z = 0.06   # head length
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        # Floating-base TF
        self.base_tf = TransformStamped()
        self.base_tf.header.frame_id = 'world'
        self.base_tf.child_frame_id = 'pelvis'

        self.get_logger().info(
            f'Jacobian test on joint: {self.test_joint_name}'
        )

    # ------------------------------------------------
    def publish_floating_base_tf(self, q):
        self.base_tf.transform.translation.x = float(q[0])
        self.base_tf.transform.translation.y = float(q[1])
        self.base_tf.transform.translation.z = float(q[2])
        self.base_tf.transform.rotation.x = float(q[3])
        self.base_tf.transform.rotation.y = float(q[4])
        self.base_tf.transform.rotation.z = float(q[5])
        self.base_tf.transform.rotation.w = float(q[6])

        self.base_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.base_tf)

    # ------------------------------------------------
    def update(self):

        # -------------------------------
        # Build configuration vector q
        # -------------------------------
        q = np.zeros(self.model.nq)

        # Floating base position (pelvis)
        q[0] = 0.2 * math.sin(0.5*self.t)   # x
        q[1] = 0.2 * math.cos(0.5*self.t)   # y
        q[2] = 0.9832                       # z fixed
        
        # Floating base orientation (pelvis)
        yaw = 0.5 * math.sin(0.5*self.t)   # base yaw oscillation
        q[3:7] = quat_from_yaw(yaw)

        # Nominal arm posture
        q[7 + self.joint_names.index('left_shoulder_pitch_joint')] = -0.5
        q[7 + self.joint_names.index('left_shoulder_roll_joint')] = 0.5
        # Motion of a joint
        q[7 + self.joint_names.index('left_elbow_joint')] = 0.8*math.sin(self.t)

        # Publish robot visualization
        self.js.header.stamp = self.get_clock().now().to_msg()
        self.js.position = q[7:].tolist()
        self.joint_pub.publish(self.js)
        self.publish_floating_base_tf(q)

        # -------------------------------
        # Forward kinematics
        # -------------------------------
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        ee_pose = self.data.oMf[self.ee_frame_id]
        p0 = ee_pose.translation

        # -------------------------------
        # Jacobian computation
        # -------------------------------
        pin.computeJointJacobians(self.model, self.data, q)

        J = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, 
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED )

        # Small joint velocity (dq)
        dq = np.zeros(self.model.nv)
        dq[6 + self.test_joint_index] = -0.5  # rad/s

        v = J @ dq
        v_linear = v[0:3]

        # -------------------------------
        # Visualize Jacobian direction
        # -------------------------------
        scale = 0.2
        p1 = p0 + scale*v_linear/(np.linalg.norm(v_linear)+1e-6)

        start = Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2]))
        end   = Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))

        self.marker.points = [start, end]
        self.marker.header.stamp = self.js.header.stamp

        self.marker_pub.publish(self.marker)

        self.t += 0.05


def main():
    rclpy.init()
    node = JacobianTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
