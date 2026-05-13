#!/usr/bin/env python3
import rclpy
import threading
import numpy as np

from pycodes_h1.functions import *
from pycodes_h1.markers import *

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# 🔹 Crear marcador de trayectoria
def create_line_marker(frame="torso_link"):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = "trajectory"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.01  # grosor línea

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.orientation.w = 1.0
    marker.points = []

    return marker


def main():

  rclpy.init()
  node = rclpy.create_node('Inverse_Arm_Kinematics')

  pub = node.create_publisher(JointState, 'joint_states', 10)
  marker_pub = node.create_publisher(Marker, 'ee_marker', 10)

  # 🔴 marcador esfera
  marker = create_sphere_marker(
      frame="torso_link",
      ns="end_effector",
      marker_id=0,
      scale=0.05,
      color=(1,0,0,1)
  )

  # 🟢 marcador trayectoria
  traj_marker = create_line_marker()

  thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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

  # 🔹 Config inicial
  q = np.zeros(7)

  # 🔹 Límites articulares
  q_min = np.array([-2.5, -2.0, -2.5, -2.5, -2.5, -2.0, -2.5])
  q_max = np.array([ 2.5,  2.0,  2.5,  2.5,  2.5,  2.0,  2.5])

  # 🔹 Pose base objetivo
  q_target = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
  T_des = fkine_arm_left_unitree(q_target)
  xd_base = TF2xyzquat(T_des)

  # 🔹 Tiempo
  t = 0.0
  dt = 0.05

  # 🔹 ROS msg
  jstate = JointState()
  jstate.name = jnames

  rate = node.create_rate(20)

  # 🔒 límite de puntos trayectoria
  max_points = 500

  while rclpy.ok():
    jstate.header.stamp = node.get_clock().now().to_msg()

    # 🔹 trayectoria deseada
    xd = xd_base.copy()
    xd[0] += 0.05 * np.sin(t)
    xd[1] += 0.05 * np.cos(t)

    # 🔹 IK
    q = ik_dls_advanced(
        fkine_arm_left_unitree,
        TF2xyzquat,
        q,
        xd,
        q_min,
        q_max,
        max_iter=1,
        lamb=0.02,
        k_null=0.2
    )

    # 🔹 publicar joints
    jstate.position = q.tolist()
    pub.publish(jstate)

    # 🔹 FK
    T = fkine_arm_left_unitree(q)
    x = TF2xyzquat(T)

    # 🔴 actualizar esfera
    marker = set_marker_pose(marker, x, node)
    marker_pub.publish(marker)

    # 🟢 agregar punto trayectoria
    p = Point()
    p.x = x[0]
    p.y = x[1]
    p.z = x[2]

    traj_marker.points.append(p)

    # 🔒 limitar tamaño
    if len(traj_marker.points) > max_points:
        traj_marker.points.pop(0)

    traj_marker.header.stamp = node.get_clock().now().to_msg()
    marker_pub.publish(traj_marker)

    # 🔹 tiempo
    t += dt
    rate.sleep()

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()