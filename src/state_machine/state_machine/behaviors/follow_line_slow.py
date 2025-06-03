#!/usr/bin/env python3
"""
Behavior para seguimiento de línea lento usando OpenCV.
"""

import cv2
import numpy as np
import py_trees
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import math

class FollowLineSlow(py_trees.behaviour.Behaviour):
    def __init__(self, name="FollowLineSlow", node=None):
        super().__init__(name)
        self.node = node
        self.bridge = CvBridge()
        
        # Parámetros de visión (más conservadores para velocidad lenta)
        self.vision_height = 15
        self.roi_center_width = 160
        self.threshold_value = 80
        self.min_area = 15
        self.max_area = 1000
        self.blur_kernel = 9
        self.morph_kernel = 7
        self.erode_iterations = 2
        self.dilate_iterations = 2
        self.use_edge_detection = True
        self.memory_factor = 0.8  # Más suavizado para velocidad lenta
        
        # Parámetros de control (más conservadores)
        self.kp = 0.2
        self.ki = 0.0
        self.kd = 0.0
        self.kp_angle = 0.05
        self.ki_angle = 0.0
        self.kd_angle = 0.0
        self.angle_weight = 0.3
        self.target_angle = 0.0
        self.linear_speed = 0.04  # Velocidad lenta
        self.max_angular_speed = 0.12
        
        # Variables de estado
        self.accumulated_error = 0.0
        self.previous_error = 0.0
        self.max_accumulated_error = 1000.0
        self.accumulated_angle_error = 0.0
        self.previous_angle_error = 0.0
        self.max_accumulated_angle_error = 1000.0
        
        self.last_valid_centroid = None
        self.last_valid_angle = 0.0
        self.last_valid_endpoints = None
        self.last_valid_error = 0.0
        self.recovery_direction = 1
        self.confidence = 0.0
        self.consecutive_detections = 0
        self.consecutive_misses = 0
        
        # Suscripciones y publicadores
        self.image_sub = None
        self.cmd_vel_pub = None
    
    def setup(self, **kwargs):
        if self.node:
            self.image_sub = self.node.create_subscription(
                CompressedImage, '/video_source/compressed',
                self.image_callback, qos_profile_sensor_data)
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("FollowLineSlow behavior configurado")
        return True
    
    def initialise(self):
        if self.node:
            self.node.get_logger().info("Iniciando seguimiento de línea lento")
    
    def update(self):
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("Terminando seguimiento de línea lento")
    
    def image_callback(self, msg: CompressedImage):
        try:
            # 1) Decodificar y rotar 180°
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.rotate(img, cv2.ROTATE_180)

            # 2) ROI inferior-centro (más grande para velocidad lenta)
            h, w = img.shape[:2]
            roi_h = min(self.vision_height, h)
            cx = w // 2
            half = self.roi_center_width // 2
            roi = img[h - roi_h: h,
                      max(0, cx - half): min(w, cx + half)]
            roi_h, roi_w = roi.shape[:2]
            local_target_x = roi_w // 2

            # 3) Preprocesamiento (más suave)
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            k = self.blur_kernel + (self.blur_kernel % 2 == 0)
            blurred = cv2.GaussianBlur(gray, (k, k), 3.0)  # Más suavizado

            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 51, 10)
            _, binary_g = cv2.threshold(blurred, self.threshold_value, 255,
                                        cv2.THRESH_BINARY_INV)
            binary = cv2.bitwise_or(binary, binary_g)

            if self.use_edge_detection:
                edges = cv2.Canny(blurred, 40, 120)  # Bordes más suaves
                edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), 1)
                binary = cv2.bitwise_or(binary, edges)

            kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
            morph = cv2.erode(binary, kernel, self.erode_iterations)
            morph = cv2.dilate(morph, kernel, self.dilate_iterations)

            # 4) Componentes conectados
            n, labels, stats, cents = cv2.connectedComponentsWithStats(morph, 8)

            best_idx, best_score = -1, -1
            for i in range(1, n):
                x, y, ww, hh, area = stats[i]
                cx_i, cy_i = cents[i]
                if self.min_area <= area <= self.max_area:
                    score = area / (abs(cx_i - local_target_x) + 1)
                    if score > best_score:
                        best_idx, best_score = i, score
                        best_cx, best_cy, best_area = cx_i, cy_i, area

            current_c, current_ang, current_eps, conf = None, 0.0, None, 0.0
            if best_idx > 0:
                current_c = np.array([best_cx, best_cy])
                mask = np.zeros_like(morph)
                mask[labels == best_idx] = 255
                cnt, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnt:
                    vx, vy, x0, y0 = cv2.fitLine(cnt[0], cv2.DIST_L2, 0, 0.01, 0.01)
                    current_ang = (math.degrees(math.atan2(vy, vx)) - 90 + 180) % 180 - 90
                    length = max(stats[best_idx][3], 30)
                    p1 = (int(x0 - vx * length), int(y0 - vy * length))
                    p2 = (int(x0 + vx * length), int(y0 + vy * length))
                    current_eps = (p1, p2)
                conf = min(1.0, best_area / (self.min_area * 3))
                self.consecutive_detections += 1
                self.consecutive_misses = 0
            else:
                self.consecutive_detections = 0
                self.consecutive_misses += 1

            # 5) Suavizado (más agresivo)
            final_c, final_ang, final_eps = None, None, None
            if current_c is not None:
                if self.last_valid_centroid is not None:
                    final_c = (1 - self.memory_factor) * current_c + self.memory_factor * self.last_valid_centroid
                    final_ang = (1 - self.memory_factor) * current_ang + self.memory_factor * self.last_valid_angle
                else:
                    final_c, final_ang = current_c, current_ang
                final_eps = current_eps
                self.last_valid_centroid, self.last_valid_angle, self.last_valid_endpoints = final_c, final_ang, final_eps
                self.confidence = conf
            elif self.last_valid_centroid is not None and self.consecutive_misses < 15:  # Más tolerante
                final_c, final_ang, final_eps = self.last_valid_centroid, self.last_valid_angle, self.last_valid_endpoints
                self.confidence = max(0.0, self.confidence - 0.05)  # Decae más lento
            else:
                self.last_valid_centroid = None
                self.confidence = 0.0

            # 6) Control
            if final_c is not None:
                pos_err = local_target_x - int(final_c[0])
                self.last_valid_error = pos_err
                ang_err = self.target_angle - final_ang
                self.apply_pid(pos_err, ang_err, self.confidence, roi_w)
            else:
                if self.consecutive_misses > 8:  # Más tolerante antes de recuperación
                    self.recovery_behavior()
                else:
                    self.stop_robot()

        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Error en seguimiento lento: {e}')
            self.stop_robot()

    def apply_pid(self, pos_err, ang_err, conf, roi_w):
        # PID posición (más suave)
        self.accumulated_error += pos_err * conf
        self.accumulated_error = max(-self.max_accumulated_error,
                                     min(self.accumulated_error, self.max_accumulated_error))
        d_pos = pos_err - self.previous_error
        self.previous_error = pos_err
        pos_w = self.kp * pos_err * conf + self.ki * self.accumulated_error + self.kd * d_pos * conf

        # PID ángulo
        self.accumulated_angle_error += ang_err * conf
        self.accumulated_angle_error = max(-self.max_accumulated_angle_error,
                                           min(self.accumulated_angle_error, self.max_accumulated_angle_error))
        d_ang = ang_err - self.previous_angle_error
        self.previous_angle_error = ang_err
        ang_w = (self.kp_angle * ang_err * conf +
                 self.ki_angle * self.accumulated_angle_error +
                 self.kd_angle * d_ang * conf)

        omega = ((1 - self.angle_weight) * pos_w +
                 self.angle_weight * ang_w)
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))

        err_mag = abs(pos_err) / (roi_w / 2)
        v = self.linear_speed * (1 - min(0.5, err_mag)) * min(1.0, conf * 1.2)  # Menos reducción de velocidad

        t = Twist()
        t.linear.x, t.angular.z = v, omega
        self.cmd_vel_pub.publish(t)

    def recovery_behavior(self):
        if self.last_valid_error != 0:
            self.recovery_direction = -1 if self.last_valid_error > 0 else 1
        t = Twist()
        t.linear.x = self.linear_speed * 0.2  # Recuperación más lenta
        t.angular.z = self.recovery_direction * self.max_angular_speed * 0.5
        self.cmd_vel_pub.publish(t)

    def stop_robot(self):
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist()) 