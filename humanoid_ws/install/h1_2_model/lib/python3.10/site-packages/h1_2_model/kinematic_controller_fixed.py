"""
 Kinematic controller for the H1-2 robot with fixed floating base (pelvis).

 It moves the left wrist to a desired target pose using an iterative
 damped least squares approach.
 It also publishes the joint states for visualization and broadcasts the TF
 from the world frame to the pelvis frame ("floating base").

 Run it as: ros2 launch h1_2_model display_handless_kinecontrol.launch.py
            ros2 run h1_2_model kinematic_controller_wb1
"""

import rclpy
from rclpy.node import Node

import numpy as np
import pinocchio as pin
import os

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class KinematicControllerFixedNode(Node):

    def __init__(self):
        super().__init__('h1_2_kinematic_controller_fixed')

        # -------------------------------
        # Load URDF
        # -------------------------------
        pkg_share = get_package_share_directory('h1_2_description')
        urdf_path = os.path.join(pkg_share, 'urdf', 'h1_2_handless.urdf')

        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        # End-effector
        self.ee_name = 'left_wrist_yaw_link'
        self.ee_id = self.model.getFrameId(self.ee_name)

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

        self.nq = self.model.nq
        self.nv = self.model.nv

        # -------------------------------
        # Initial configuration
        # -------------------------------
        self.q = pin.neutral(self.model)
        # Fixed floating base
        self.q[0:3] = np.array([0.0, 0.0, 0.9832])
        self.q[3:7] = np.array([0.0, 0.0, 0.0, 1.0])

        # -------------------------------
        # Target pose (SE3)
        # -------------------------------
        R_target = pin.utils.rpyToMatrix(0.0, 0.0, 0.0)
        p_target = np.array([0.4+0.2, 0.25+0.1, 1.1+0.2])
        self.T_target = pin.SE3(R_target, p_target)

        # -------------------------------
        # ROS publishers
        # -------------------------------
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/ik_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update)

        # Parameters for the kinematic controller
        self.alpha = 0.04
        self.damping = 1e-6

        # Boolean flag
        self.ik_converged = False


    # ------------------------------------------------
    # Main loop
    # ------------------------------------------------
    def update(self):
        # Forward kinematics
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        T_current = self.data.oMf[self.ee_id]

        # Error twist
        error_se3 = pin.log(T_current.inverse() * self.T_target)
        error = error_se3.vector

        # Convergence check  
        if np.linalg.norm(error) < 1e-4:
            if not self.ik_converged:
                self.get_logger().info("Kinematic controller converged")
                self.ik_converged = True
            return

        # Jacobian
        pin.computeJointJacobians(self.model, self.data, self.q)
        J = pin.getFrameJacobian(self.model, self.data, self.ee_id, 
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # Fix the floating base (remove floating base columns and only use actuated joints)
        J_act = J[:, 6:]

        # Damped least squares
        JJt = J_act @ J_act.T
        dq = J_act.T @ np.linalg.solve(JJt + self.damping*np.eye(6), error)

        # Integrate the actuated joints (keep the fixed base unchanged)
        self.q[7:] += self.alpha*dq

        # Publish
        self.publish_joint_state()
        self.publish_markers(T_current)

    # ------------------------------------------------
    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q[7:].tolist()
        self.js_pub.publish(msg)

        # world -> pelvis TF
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'pelvis'
        tf.transform.translation.x = self.q[0]
        tf.transform.translation.y = self.q[1]
        tf.transform.translation.z = self.q[2]
        tf.transform.rotation.x = self.q[3]
        tf.transform.rotation.y = self.q[4]
        tf.transform.rotation.z = self.q[5]
        tf.transform.rotation.w = self.q[6]
        self.tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------
    def publish_markers(self, T_current):
        # Target marker (green)
        self.publish_sphere(self.T_target.translation, [0.0, 1.0, 0.0], 0)

        # Current EE marker (red)
        self.publish_sphere(T_current.translation, [1.0, 0.0, 0.0], 1)

    def publish_sphere(self, p, color, marker_id):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'ik'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.06
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2])
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = KinematicControllerFixedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
