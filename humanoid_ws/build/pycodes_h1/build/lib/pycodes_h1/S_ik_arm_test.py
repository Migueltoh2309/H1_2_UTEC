#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
from pycodes_h1.functions import *
from pycodes_h1.markers import *
from sensor_msgs.msg import JointState
from copy import copy

def main():

  rclpy.init()
  node = rclpy.create_node('Inverse_Arm_Kinematics')
  pub = node.create_publisher(JointState, 'joint_states', 10)
  marker_pub = node.create_publisher(Marker, 'ee_marker', 10)

  marker = create_sphere_marker(
              frame="torso_link",
              ns="end_effector",
              marker_id=0,
              scale=0.05,
              color=(1,0,0,1))

  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()
 
  # Joint names
  jnames = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint"
        ]

  # Config inicial
  q = np.zeros(7)

  # Pose base objetivo
  q_target = np.array([-np.pi/6, np.pi/6, 0.0, np.pi/2, 0.0, -np.pi/6, 0.0])
  T_des = fkine_arm_left_unitree(q_target)
  xd_base = TF2xyzquat(T_des)

  # Tiempo
  t = 0.0
  dt = 0.05  # consistente con 20 Hz

  # Object (message)
  jstate = JointState()
  jstate.name = jnames
  
  rate = node.create_rate(20)

  while rclpy.ok():
    jstate.header.stamp = node.get_clock().now().to_msg()

    # 🔹 Copiar pose base
    xd = xd_base.copy()

    # 🔥 Movimiento sinusoidal en X
    xd[0] += 0.001 * np.sin(t)

    # IK incremental
    q = ik_dls(
        fkine_arm_left_unitree,
        TF2xyzquat,
        q,
        xd,
        max_iter=1
    )

    # Publicar joints
    jstate.position = q.tolist()
    pub.publish(jstate)

    # FK para visualizar
    T = fkine_arm_left_unitree(q)
    x = TF2xyzquat(T)

    marker = set_marker_pose(marker, x, node)
    marker_pub.publish(marker)

    # 🔹 actualizar tiempo
    t += dt

    rate.sleep()
  
  node.destroy_node()
  rclpy.shutdown()