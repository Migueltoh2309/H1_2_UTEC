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
  node = rclpy.create_node('Forward_Arm_Kinematics')
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
  # Joint Configuration
  q  = np.array([-3.14, -0.38, -3.01, -2.53, -2.967, -0.471, -1.102])
 
  # End effector with respect to the base
  T = fkine_arm_left_unitree(q)
  print(np.round(T,3))

  x0 = TF2xyzquat(T)

  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  # Add the head joint value (with value 0) to the joints
  jstate.position = q.tolist()
  
  # Loop rate (in Hz)
  rate = node.create_rate(20)
  # Continuous execution loop
  while rclpy.ok():
    # Current time (needed for ROS)
    jstate.header.stamp = node.get_clock().now().to_msg()

    # Publish joint states
    pub.publish(jstate)

    # calcular pose del end-effector
    T = fkine_arm_left_unitree(q)
    x = TF2xyzquat(T)

    # actualizar marker
    marker = set_marker_pose(marker, x, node)

    # publicar marker
    marker_pub.publish(marker)

    # Wait for the next iteration
    rate.sleep()
  
  node.destroy_node()
  rclpy.shutdown()
 