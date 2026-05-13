"""
Jacobian Numerical Test Node for the H1-2 Robot

This node computes the Jacobian of the H1-2 robot using Pinocchio and compares 
it against a numerically computed Jacobian using finite differences. It reports 
the maximum, mean, and Frobenius norm of the error between the two Jacobians.

Run it as: ros2 run h1_2_model jacobian_numeric_test
"""


import rclpy
from rclpy.node import Node

import numpy as np
import pinocchio as pin
import os

from ament_index_python.packages import get_package_share_directory


class JacobianNumericTestNode(Node):

    def __init__(self):
        super().__init__('h1_2_jacobian_numeric_test')

        # -------------------------------
        # Load URDF into Pinocchio
        # -------------------------------
        pkg_share = get_package_share_directory('h1_2_description')
        urdf_path = os.path.join(pkg_share, 'urdf', 'h1_2_handless.urdf')
        
        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        self.ee_frame = 'left_wrist_yaw_link'
        self.ee_frame_id = self.model.getFrameId(self.ee_frame)
        self.get_logger().info(f'End-effector frame "{self.ee_frame}" id = {self.ee_frame_id}')

        # Run test once
        self.run_test()

        # Shutdown cleanly
        rclpy.shutdown()


    def run_test(self):
        """ 
        Run the numerical Jacobian test
        """
        model = self.model
        data = self.data
        ee_id = self.ee_frame_id

        nq = model.nq
        nv = model.nv
        self.get_logger().info(f'nq = {nq}, nv = {nv}')

        # ----------------------------------
        # Joint configuration
        # ----------------------------------
        q = pin.neutral(model)

        # Floating base pose
        q[0:3] = np.array([0.0, 0.0, 0.9832])
        q[3:7] = np.array([0.0, 0.0, 0.0, 1.0])

        # Some non-zero joint configuration
        if nv > 6:
            q[7:] = 0.1

        # ----------------------------------
        # Forward kinematics at q
        # ----------------------------------
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        oMf0 = data.oMf[ee_id]
        p0 = oMf0.translation.copy()
        R0 = oMf0.rotation.copy()

        # ----------------------------------------------
        # Geometric Jacobian: mapping qdot -> (v, omega)
        # ----------------------------------------------
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        J = pin.getFrameJacobian(model, data, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # ----------------------------------------
        # Geometric Jacobian computed numerically
        # ----------------------------------------
        delta = 1e-8
        J_num = np.zeros_like(J)

        for i in range(nv):
            dq = np.zeros(nv)
            dq[i] = 1.0

            # Next q using a delta (integration)
            q_plus = pin.integrate(model, q, delta*dq)

            # Forward kinematics at q_plus
            pin.forwardKinematics(model, data, q_plus)
            pin.updateFramePlacements(model, data)
            oMf_plus = data.oMf[ee_id]
            p_plus = oMf_plus.translation
            R_plus = oMf_plus.rotation

            # Linear velocity (finite difference)
            dp = (p_plus-p0)/delta

            # Angular velocity (using SO(3) log)
            dR = R_plus @ R0.T
            domega = pin.log3(dR)/delta

            J_num[0:3, i] = dp
            J_num[3:6, i] = domega

        # ----------------------------------
        # Error analysis
        # ----------------------------------
        error = J - J_num

        max_err = np.max(np.abs(error))
        mean_err = np.mean(np.abs(error))
        norm_err = np.linalg.norm(error)

        self.get_logger().info('==== Jacobian Numerical Test ====')
        self.get_logger().info(f'Max absolute error   : {max_err:.3e}')
        self.get_logger().info(f'Mean absolute error  : {mean_err:.3e}')
        self.get_logger().info(f'Frobenius norm error : {norm_err:.3e}')

        # Per-column check
        worst_col = np.argmax(np.linalg.norm(error, axis=0))
        self.get_logger().info(f'Worst column index: {worst_col}')
        self.get_logger().info('================================')

        # Sanity assertion
        if max_err > 1e-5:
            self.get_logger().warn('Jacobian numerical test FAILED (error too large)')
        else:
            self.get_logger().info('Jacobian numerical test PASSED')


def main():
    rclpy.init()
    JacobianNumericTestNode()
