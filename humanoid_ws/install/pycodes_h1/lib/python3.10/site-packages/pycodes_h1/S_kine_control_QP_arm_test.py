#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import osqp
import scipy.sparse as sp

from pycodes_h1.functions import *
from pycodes_h1.markers import *

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():

    rclpy.init()
    node = rclpy.create_node('QP_Operational_Control')

    pub = node.create_publisher(JointState, 'joint_states', 10)
    marker_pub = node.create_publisher(Marker, 'ee_marker', 10)

    marker = create_sphere_marker(
        frame="torso_link",
        ns="end_effector",
        marker_id=0,
        scale=0.05,
        color=(1,0,0,1)
    )

    traj_marker = create_line_marker()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    jnames = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint"
    ]

    q = np.zeros(7)

    q_min = np.array([-3.14, -0.38, -3.01, -2.53, -2.967, -0.471, -1.102])
    q_max = np.array([ 1.57,  3.4,  2.66,  1.6,  2.967,  0.349,  1.102])

    # Pose base
    q_target = np.zeros(7)
    T_base = fkine_arm_left_unitree(q_target)

    t = 0.0
    dt = 0.05
    omega = 1.0
    r = 0.05    

    jstate = JointState()
    jstate.name = jnames

    rate = node.create_rate(20)

    max_points = 500

    while rclpy.ok():
        jstate.header.stamp = node.get_clock().now().to_msg()

        # 🔵 Trayectoria
        Td = T_base.copy()

        Td[0,3] += r * np.cos(t)
        Td[1,3] += r * np.cos(t) + r * np.sin(t)
        Td[2,3] += r * np.sin(t)

        # 🔵 Orientación dinámica (rotación en Z)
        theta = 0.5 * t

        Rz = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

        Td[:3,:3] = Rz @ T_base[:3,:3]

        # 🔥 QP Control
        dq = qp_differential_ik_osqp(
            fkine_arm_left_unitree,
            q,
            Td,
            q_min,
            q_max,
            dt,
            K=2.0,
            lambda_reg=1e-4
        )

        # 🔹 Integración
        q = q + dq * dt


        # 🔹 Saturación
        q = np.clip(q, q_min, q_max)
        print(f"q1: {q[0]:.3f} rad | q2: {q[1]:.3f} rad")
        
        # 🔹 Publicar joints
        jstate.position = q.tolist()
        pub.publish(jstate)

        # 🔹 FK actual
        T = fkine_arm_left_unitree(q)
        x = TF2xyzquat(T)

        # 🔴 esfera
        marker = set_marker_pose(marker, x, node)
        marker_pub.publish(marker)

        # 🟢 trayectoria
        p = Point()
        p.x = x[0]
        p.y = x[1]
        p.z = x[2]

        traj_marker.points.append(p)

        if len(traj_marker.points) > max_points:
            traj_marker.points.pop(0)

        traj_marker.header.stamp = node.get_clock().now().to_msg()
        marker_pub.publish(traj_marker)

        t += omega * dt
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()