#!/usr/bin/env python3
"""
Behavior mejorado para seguimiento de línea lento usando OpenCV.
Implementa lógica avanzada con componentes conectados, control PID dual y sistema de confianza.
"""

import cv2
import numpy as np
import py_trees
import math
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge


class FollowLineSlow(py_trees.behaviour.Behaviour):
    def __init__(self, name="FollowLineSlow", node=None):
        super().__init__(name)
        self.node = node
        self.bridge = CvBridge()

        # -----------------------------------------------------------
        # Parámetros mejorados para seguimiento robusto
        # -----------------------------------------------------------
        self.vision_height = 15        # Altura del ROI inferior-central
        self.roi_center_width = 140    # Ancho del ROI
        self.threshold_value = 80      # Valor de umbralización
        self.min_area = 15            # Área mínima de componente
        self.max_area = 1000          # Área máxima de componente
        self.blur_kernel = 9          # Kernel de suavizado
        self.morph_kernel = 7         # Kernel morfológico
        self.erode_iterations = 3     # Iteraciones de erosión
        self.dilate_iterations = 3    # Iteraciones de dilatación
        self.use_edge_detection = True # Usar detección de bordes
        self.memory_factor = 0.7      # Factor de suavizado temporal

        # Control PID dual (posición + ángulo)
        self.kp = 0.003
        self.ki = 0.0001
        self.kd = 0.0
        self.kp_angle = 0.0
        self.ki_angle = 0.0
        self.kd_angle = 0.0
        self.angle_weight = 0.4       # Peso del control angular
        self.target_angle = 0.0       # Ángulo objetivo

        # Velocidades para modo lento
        self.linear_speed = 0.05      # Velocidad lineal base
        self.max_angular_speed = 0.12 # Velocidad angular máxima
        self.recovery_mode = True     # Modo de recuperación activo

        # -----------------------------------------------------------
        # Variables de estado PID
        # -----------------------------------------------------------
        self.accumulated_error = 0.0
        self.previous_error = 0.0
        self.max_accumulated_error = 1000.0
        self.accumulated_angle_error = 0.0
        self.previous_angle_error = 0.0
        self.max_accumulated_angle_error = 1000.0

        # Variables de memoria y confianza
        self.last_valid_centroid = None
        self.last_valid_angle = 0.0
        self.last_valid_endpoints = None
        self.last_valid_error = 0.0
        self.recovery_direction = 1
        self.confidence = 0.0
        self.consecutive_detections = 0
        self.consecutive_misses = 0

        # Factor de escala para debug
        self.scale_factor = 2.0

        # Suscripción y publicador
        self.image_sub = None
        self.cmd_vel_pub = None

        # Flag de actividad
        self.is_active = False

        # Último frame recibido
        self.latest_frame = None

    def setup(self, **kwargs):
        """
        Se llama una sola vez al configurar el árbol. Creamos suscripción y publicador.
        """
        if self.node:
            # Publicador para cmd_vel
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

            # Suscripción al tópico de la cámara
            self.image_sub = self.node.create_subscription(
                CompressedImage,
                '/video_source/compressed',
                self.image_callback,
                qos_profile_sensor_data
            )
            self.node.get_logger().info("🚀 FollowLineSlow: suscripción a /video_source/compressed creada.")
        return True

    def initialise(self):
        """
        Se llama cada vez que el árbol entra en este comportamiento.
        Activamos el flag y reiniciamos variables.
        """
        if self.node:
            self.node.get_logger().info("🚀 FollowLineSlow: iniciando seguimiento de línea lento avanzado")

        self.is_active = True

        # Reset estado PID y variables de memoria
        self.accumulated_error = 0.0
        self.previous_error = 0.0
        self.accumulated_angle_error = 0.0
        self.previous_angle_error = 0.0
        self.last_valid_centroid = None
        self.last_valid_angle = 0.0
        self.last_valid_endpoints = None
        self.last_valid_error = 0.0
        self.recovery_direction = 1
        self.confidence = 0.0
        self.consecutive_detections = 0
        self.consecutive_misses = 0

        # Crear ventanas de debug
        cv2.namedWindow('Debug - Original', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Debug - Procesado', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Debug - Control', cv2.WINDOW_NORMAL)
        if self.use_edge_detection:
            cv2.namedWindow('Debug - Bordes', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Debug - Original', 400, 300)
        cv2.resizeWindow('Debug - Procesado', 400, 300)
        cv2.resizeWindow('Debug - Control', 400, 300)

    def show_debug(self, name, frame, scale=None):
        """Muestra la imagen reescalada para debug."""
        if scale is None:
            scale = self.scale_factor
        if scale != 1.0:
            frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(name, frame)

    def apply_pid_control(self, pos_error, angle_error, confidence, roi_width):
        """Control PID dual: posición + ángulo"""
        # PID para posición
        self.accumulated_error += pos_error * confidence
        self.accumulated_error = max(-self.max_accumulated_error,
                                     min(self.accumulated_error, self.max_accumulated_error))
        d_pos = pos_error - self.previous_error
        self.previous_error = pos_error
        pos_control = (self.kp * pos_error * confidence + 
                      self.ki * self.accumulated_error + 
                      self.kd * d_pos * confidence)

        # PID para ángulo
        self.accumulated_angle_error += angle_error * confidence
        self.accumulated_angle_error = max(-self.max_accumulated_angle_error,
                                          min(self.accumulated_angle_error, self.max_accumulated_angle_error))
        d_angle = angle_error - self.previous_angle_error
        self.previous_angle_error = angle_error
        angle_control = (self.kp_angle * angle_error * confidence +
                        self.ki_angle * self.accumulated_angle_error +
                        self.kd_angle * d_angle * confidence)

        # Combinar controles
        omega = ((1 - self.angle_weight) * pos_control +
                 self.angle_weight * angle_control)
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))

        # Velocidad adaptativa basada en error
        error_magnitude = abs(pos_error) / (roi_width / 2)
        linear_velocity = self.linear_speed * (1 - min(0.7, error_magnitude)) * min(1.0, confidence * 1.2)

        # Publicar comando
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = omega
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)

    def recovery_behavior(self):
        """Comportamiento de recuperación cuando se pierde la línea"""
        if self.last_valid_error != 0:
            self.recovery_direction = -1 if self.last_valid_error > 0 else 1
        
        twist = Twist()
        twist.linear.x = self.linear_speed * 0.3
        twist.angular.z = self.recovery_direction * self.max_angular_speed * 0.7
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Detener el robot completamente"""
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist())

    def update(self):
        """
        Procesa el último frame recibido y aplica la lógica avanzada de seguimiento.
        """
        if self.latest_frame is None:
            return py_trees.common.Status.RUNNING

        try:
            # 1) Rotar imagen 180° y extraer ROI
            img = self.latest_frame.copy()
            h, w = img.shape[:2]
            
            # ROI inferior-central limitado por altura real
            roi_h = min(self.vision_height, h)
            cx = w // 2
            half = self.roi_center_width // 2
            roi = img[h - roi_h: h,
                      max(0, cx - half): min(w, cx + half)]
            roi_h, roi_w = roi.shape[:2]
            local_target_x = roi_w // 2

            original_view = roi.copy()

            # 2) Preprocesamiento avanzado
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            k = self.blur_kernel + (self.blur_kernel % 2 == 0)
            blurred = cv2.GaussianBlur(gray, (k, k), 2.0)

            # Umbralización adaptativa + global
            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 51, 10)
            _, binary_global = cv2.threshold(blurred, self.threshold_value, 255,
                                           cv2.THRESH_BINARY_INV)
            binary = cv2.bitwise_or(binary, binary_global)

            # Detección de bordes opcional
            if self.use_edge_detection:
                edges = cv2.Canny(blurred, 50, 150)
                edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), 1)
                binary = cv2.bitwise_or(binary, edges)
                edge_view = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            # Operaciones morfológicas
            kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
            morph = cv2.erode(binary, kernel, self.erode_iterations)
            morph = cv2.dilate(morph, kernel, self.dilate_iterations)

            # 3) Análisis de componentes conectados
            n_components, labels, stats, centroids = cv2.connectedComponentsWithStats(morph, 8)

            # Encontrar el mejor componente
            best_idx, best_score = -1, -1
            for i in range(1, n_components):
                x, y, width, height, area = stats[i]
                cx_component, cy_component = centroids[i]
                
                if self.min_area <= area <= self.max_area:
                    # Puntaje basado en área y cercanía al centro
                    score = area / (abs(cx_component - local_target_x) + 1)
                    if score > best_score:
                        best_idx, best_score = i, score
                        best_cx, best_cy, best_area = cx_component, cy_component, area

            # 4) Procesamiento del componente encontrado
            current_centroid, current_angle, current_endpoints, conf = None, 0.0, None, 0.0
            
            if best_idx > 0:
                current_centroid = np.array([best_cx, best_cy])
                
                # Crear máscara del componente y calcular línea
                mask = np.zeros_like(morph)
                mask[labels == best_idx] = 255
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    # Ajustar línea y calcular ángulo
                    vx, vy, x0, y0 = cv2.fitLine(contours[0], cv2.DIST_L2, 0, 0.01, 0.01)
                    current_angle = (math.degrees(math.atan2(vy, vx)) - 90 + 180) % 180 - 90
                    
                    # Calcular endpoints para visualización
                    length = max(stats[best_idx][3], 30)
                    p1 = (int(x0 - vx * length), int(y0 - vy * length))
                    p2 = (int(x0 + vx * length), int(y0 + vy * length))
                    current_endpoints = (p1, p2)
                
                # Calcular confianza basada en área
                conf = min(1.0, best_area / (self.min_area * 3))
                self.consecutive_detections += 1
                self.consecutive_misses = 0
            else:
                self.consecutive_detections = 0
                self.consecutive_misses += 1

            # 5) Suavizado temporal con memoria
            final_centroid, final_angle, final_endpoints = None, None, None
            
            if current_centroid is not None:
                if self.last_valid_centroid is not None:
                    # Suavizado exponencial
                    final_centroid = ((1 - self.memory_factor) * current_centroid + 
                                    self.memory_factor * self.last_valid_centroid)
                    final_angle = ((1 - self.memory_factor) * current_angle + 
                                 self.memory_factor * self.last_valid_angle)
                else:
                    final_centroid, final_angle = current_centroid, current_angle
                
                final_endpoints = current_endpoints
                self.last_valid_centroid = final_centroid
                self.last_valid_angle = final_angle
                self.last_valid_endpoints = final_endpoints
                self.confidence = conf
                
            elif self.last_valid_centroid is not None and self.consecutive_misses < 10:
                # Usar último valor válido por un tiempo limitado
                final_centroid = self.last_valid_centroid
                final_angle = self.last_valid_angle
                final_endpoints = self.last_valid_endpoints
                self.confidence = max(0.0, self.confidence - 0.1)
            else:
                # Perder completamente la línea
                self.last_valid_centroid = None
                self.confidence = 0.0

            # 6) Control del robot
            if final_centroid is not None:
                # Calcular errores
                pos_error = local_target_x - int(final_centroid[0])
                angle_error = self.target_angle - final_angle
                self.last_valid_error = pos_error
                
                # Aplicar control PID dual
                self.apply_pid_control(pos_error, angle_error, self.confidence, roi_w)
            else:
                # Sin línea detectada
                if self.recovery_mode and self.consecutive_misses > 5:
                    self.recovery_behavior()
                else:
                    self.stop_robot()

            # 7) Debug visual cada 3 frames
            if hasattr(self, 'frame_count'):
                self.frame_count += 1
            else:
                self.frame_count = 0
                
            if self.frame_count % 3 == 0:
                self.show_debug('Debug - Original', original_view)
                self.show_debug('Debug - Procesado', cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR))
                
                if final_centroid is not None:
                    control_view = original_view.copy()
                    cv2.circle(control_view, (int(final_centroid[0]), int(final_centroid[1])), 6, (0, 0, 255), -1)
                    cv2.line(control_view, (local_target_x, 0), (local_target_x, roi_h), (255, 0, 0), 2)
                    cv2.putText(control_view, 'SLOW MODE', (10, 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    if final_endpoints:
                        cv2.line(control_view, final_endpoints[0], final_endpoints[1], (0, 255, 0), 2)
                    self.show_debug('Debug - Control', control_view)
                else:
                    self.show_debug('Debug - Control', original_view)
                
                if self.use_edge_detection and 'edge_view' in locals():
                    self.show_debug('Debug - Bordes', edge_view)
                
                cv2.waitKey(1)

        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"FollowLineSlow: error en procesamiento: {e}")
            self.stop_robot()

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Al salir de este comportamiento, desactivamos flag y detenemos el robot.
        """
        self.is_active = False
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("🛑 FollowLineSlow: terminado, robot detenido")

    def image_callback(self, msg):
        """
        Callback de ROS 2: guarda el último frame en self.latest_frame solo si is_active == True.
        """
        if not self.is_active:
            return

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.latest_frame = cv_image
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"FollowLineSlow: error al convertir imagen: {e}")
