import cv2
import mediapipe as mp
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# =======================
# FUNCIONES AUXILIARES
# =======================
def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def is_v_sign(hand_landmarks):
    index_tip = hand_landmarks.landmark[8]
    index_pip = hand_landmarks.landmark[6]

    middle_tip = hand_landmarks.landmark[12]
    middle_pip = hand_landmarks.landmark[10]

    ring_tip = hand_landmarks.landmark[16]
    ring_pip = hand_landmarks.landmark[14]

    pinky_tip = hand_landmarks.landmark[20]
    pinky_pip = hand_landmarks.landmark[18]

    return (index_tip.y < index_pip.y and
            middle_tip.y < middle_pip.y and
            ring_tip.y > ring_pip.y and
            pinky_tip.y > pinky_pip.y)

def is_rock_sign(hand_landmarks):
    index_tip = hand_landmarks.landmark[8]
    index_pip = hand_landmarks.landmark[6]

    middle_tip = hand_landmarks.landmark[12]
    middle_pip = hand_landmarks.landmark[10]

    ring_tip = hand_landmarks.landmark[16]
    ring_pip = hand_landmarks.landmark[14]

    pinky_tip = hand_landmarks.landmark[20]
    pinky_pip = hand_landmarks.landmark[18]

    return (index_tip.y < index_pip.y and
            pinky_tip.y < pinky_pip.y and
            middle_tip.y > middle_pip.y and
            ring_tip.y > ring_pip.y)

# =======================
# NODO ROS2
# =======================
class VisionPublisher(Node):

    def __init__(self):
        super().__init__('vision_publisher')

        self.publisher_ = self.create_publisher(Point, 'wrist_position', 10)

        # Cámara
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")
            exit()

        # MediaPipe
        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils

        # Estados
        self.control_active = False
        self.gesture_counter_on = 0
        self.gesture_counter_off = 0

        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def run(self):
        while rclpy.ok():

            ret, frame = self.cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            height, width, _ = frame.shape

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.holistic.process(frame_rgb)

            # =======================
            # ACTIVACIÓN
            # =======================
            if results.right_hand_landmarks:
                if is_v_sign(results.right_hand_landmarks):
                    self.gesture_counter_on += 1
                    if self.gesture_counter_on > 5:
                        self.control_active = True
                else:
                    self.gesture_counter_on = 0
            else:
                self.gesture_counter_on = 0

            # =======================
            # DESACTIVACIÓN
            # =======================
            if results.left_hand_landmarks:
                if is_rock_sign(results.left_hand_landmarks):
                    self.gesture_counter_off += 1
                    if self.gesture_counter_off > 5:
                        self.control_active = False
                else:
                    self.gesture_counter_off = 0
            else:
                self.gesture_counter_off = 0

            # =======================
            # TEXTO
            # =======================
            text = "ACTIVO" if self.control_active else "INACTIVO"
            color = (0,255,0) if self.control_active else (0,0,255)

            cv2.putText(frame, text, (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # =======================
            # POSE
            # =======================
            if self.control_active and results.pose_landmarks:

                pose = results.pose_landmarks.landmark

                x1 = int(pose[self.mp_holistic.PoseLandmark.LEFT_SHOULDER].x * width)
                y1 = int(pose[self.mp_holistic.PoseLandmark.LEFT_SHOULDER].y * height)

                x2 = int(pose[self.mp_holistic.PoseLandmark.LEFT_ELBOW].x * width)
                y2 = int(pose[self.mp_holistic.PoseLandmark.LEFT_ELBOW].y * height)

                x3 = int(pose[self.mp_holistic.PoseLandmark.LEFT_WRIST].x * width)
                y3 = int(pose[self.mp_holistic.PoseLandmark.LEFT_WRIST].y * height)

                # =======================
                # POSICIÓN
                # =======================
                L1 = distance((x1,y1),(x2,y2))
                L2 = distance((x2,y2),(x3,y3))
                L = L1 + L2

                if L > 0:
                    scale = 58.0 / L

                    x_cm = (x3 - x1) * scale
                    y_cm = -(y3 - y1) * scale

                    # 🔥 PUBLICAR
                    msg = Point()
                    msg.x = float(x_cm)
                    msg.y = float(y_cm)
                    msg.z = 0.0

                    self.publisher_.publish(msg)

                    # Debug en pantalla
                    cv2.putText(frame,
                                f"X:{x_cm:.1f} Y:{y_cm:.1f}",
                                (20, height-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255,255,255), 2)

            cv2.imshow("Vision Node", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rclpy.spin_once(self, timeout_sec=0)

        self.cap.release()
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = VisionPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()