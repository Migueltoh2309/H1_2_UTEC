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
  node = rclpy.create_node('Forward_Body_Kinematics')
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
            "left_wrist_yaw_joint",
            
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint"
        ]
  # Joint Configuration
  q_left  = np.array([-np.pi/6, np.pi/6, 0.0, np.pi/2, 0.0, -np.pi/6, 0.0])
  q_right  = np.array([-np.pi/6, np.pi/6, 0.0, np.pi/2, 0.0, -np.pi/6, 0.0])
 
  q = np.concatenate((q_left, q_right)) 

  # End effector with respect to the base
  T_left = fkine_arm_left_unitree(q_left)
  T_right = fkine_arm_right_unitree(q_right)
  print(np.round(T_left,3))
  print(np.round(T_right,3))

  x0_left = TF2xyzquat(T_left)
  x0_right = TF2xyzquat(T_right)

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
    T_left = fkine_arm_left_unitree(q_left)
    T_rigth = fkine_arm_right_unitree(q_right)
    x_left = TF2xyzquat(T_left)
    x_right = TF2xyzquat(T_rigth)

    # actualizar marker
    marker_left = set_marker_pose(marker, x_left, node)
    marker_right = set_marker_pose(marker, x_right, node)

    # publicar marker
    marker_pub.publish(marker_left)

    # Wait for the next iteration
    rate.sleep()
  
  node.destroy_node()
  rclpy.shutdown()
 