#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import threading
import sys
import termios
import tty

from sensor_msgs.msg import JointState

from h1_2_algoritms.fk_functions import *
from h1_2_algoritms.ik_functions import *
from h1_2_algoritms.markers import *


def get_key():
    """Leer una tecla sin bloquear ROS"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch


class BimanualTeleop(Node):

    def __init__(self):
        super().__init__('bimanual_teleop')

        # ===== CONFIG =====
        self.step = 0.02
        self.use_dls = True

        # ===== Publishers =====
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'ee_marker', 10)

        # ===== Joint names =====
        self.  jnames = [
                            # LEFT ARM
                            "left_shoulder_pitch_joint",
                            "left_shoulder_roll_joint",
                            "left_shoulder_yaw_joint",
                            "left_elbow_joint",
                            "left_wrist_roll_joint",
                            "left_wrist_pitch_joint",
                            "left_wrist_yaw_joint",
                            
                            # RIGHT ARM
                            "right_shoulder_pitch_joint",
                            "right_shoulder_roll_joint",
                            "right_shoulder_yaw_joint",
                            "right_elbow_joint",
                            "right_wrist_roll_joint",
                            "right_wrist_pitch_joint",
                            "right_wrist_yaw_joint"
                        ]

        # ===== ESTADO =====
        self.q_left = np.zeros(7)
        self.q_right = np.zeros(7)

        # ===== TARGETS = POSICIÓN ACTUAL =====
        T_left = fkine_arm_left_unitree(self.q_left)
        T_right = fkine_arm_right_unitree(self.q_right)

        self.targets = {
            "left": TF2xyzquat(T_left),
            "right": TF2xyzquat(T_right)
        }

        self.arms = {
            "left": {"q": self.q_left, "fk": fkine_arm_left_unitree},
            "right": {"q": self.q_right, "fk": fkine_arm_right_unitree}
        }

        # ===== MARKERS =====
        self.markers = {
            "left": create_sphere_marker("torso_link","ee",0,0.05,(1,0,0,1)),
            "right": create_sphere_marker("torso_link","ee",1,0.05,(0,0,1,1))
        }

        # ===== MSG =====
        self.jstate = JointState()
        self.jstate.name = self.jnames

        # ===== TIMER =====
        self.timer = self.create_timer(1/20.0, self.update)

        # ===== THREAD TECLADO =====
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Teleop IK iniciado 🚀")

    # =========================================
    # 🔹 TECLADO
    # =========================================
    def keyboard_loop(self):
        while True:
            key = get_key()

            # LEFT ARM
            if key == 'w': self.targets["left"][0] += self.step
            elif key == 's': self.targets["left"][0] -= self.step
            elif key == 'a': self.targets["left"][1] += self.step
            elif key == 'd': self.targets["left"][1] -= self.step
            elif key == 'q': self.targets["left"][2] += self.step
            elif key == 'e': self.targets["left"][2] -= self.step

            # RIGHT ARM
            elif key == 'i': self.targets["right"][0] += self.step
            elif key == 'k': self.targets["right"][0] -= self.step
            elif key == 'j': self.targets["right"][1] += self.step
            elif key == 'l': self.targets["right"][1] -= self.step
            elif key == 'u': self.targets["right"][2] += self.step
            elif key == 'o': self.targets["right"][2] -= self.step

            elif key == 'x':
                self.get_logger().info("Saliendo...")
                rclpy.shutdown()
                break

    # =========================================
    # 🔹 IK
    # =========================================
    def compute_ik(self, fk, q, xd):
        if self.use_dls:
            return ik_dls_step(fk, TF2xyzquat, q, xd, lamb=0.05)
        else:
            return ik_pseudo_step(fk, TF2xyzquat, q, xd, alpha=0.3)

    # =========================================
    # 🔹 LOOP
    # =========================================
    def update(self):

        # IK por brazo
        for name, arm in self.arms.items():

            q = arm["q"]
            fk = arm["fk"]
            xd = self.targets[name]

            q_new = self.compute_ik(fk, q, xd)

            if not np.any(np.isnan(q_new)):
                arm["q"][:] = q_new

        # publicar joints
        q_all = np.concatenate([self.q_left, self.q_right])

        self.jstate.header.stamp = self.get_clock().now().to_msg()
        self.jstate.position = q_all.tolist()
        self.pub.publish(self.jstate)

        # markers
        for name, arm in self.arms.items():
            T = arm["fk"](arm["q"])
            x = TF2xyzquat(T)

            if not np.any(np.isnan(x)):
                marker = set_marker_pose(self.markers[name], x, self)
                self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = BimanualTeleop()
    rclpy.spin(node)


if __name__ == "__main__":
    main()