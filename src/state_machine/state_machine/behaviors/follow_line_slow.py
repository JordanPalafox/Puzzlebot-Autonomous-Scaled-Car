#!/usr/bin/env python3
"""
Behavior robusto para seguimiento de línea lento usando OpenCV.
Basado en el mismo patrón de FollowLineFast, pero con parámetros ajustados para velocidad lenta.
"""

import cv2
import numpy as np
import py_trees
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

class FollowLineSlow(py_trees.behaviour.Behaviour):
    def __init__(self, name="FollowLineSlow", node=None):
        super().__init__(name)
        self.node = node
        self.bridge = CvBridge()

        # Parámetros para velocidad lenta
<<<<<<< HEAD
        self.roi_height = 30        # ROI más pequeño para respuesta lenta
        self.roi_width = 200        # ROI más estrecho
        self.threshold = 75
        self.kp = 0.2             # Más agresivo
        self.ki = 0.05             # Integral para eliminar error residual
        self.kd = 0.2             # Derivativo para suavizar
        self.linear_speed = 0.06   # Velocidad lenta
        self.max_angular = 0.18     # Giros más agresivos
        self.smoothing = 0.7       # Menos suavizado para respuesta lenta
        
        # Variables de estado simplificadas
=======
        self.roi_height = 30
        self.roi_width = 200
        self.threshold = 75
        self.kp = 0.2
        self.ki = 0.05
        self.kd = 0.2
        self.linear_speed = 0.06
        self.max_angular = 0.10
        self.smoothing = 0.7

        # Variables de estado
>>>>>>> 8b3b307 (giros y maquina de estados bien hecha)
        self.last_error = 0.0
        self.smoothed_error = 0.0
        self.integral_error = 0.0
        self.max_integral = 1.0
        self.last_centroid = None
        self.frame_count = 0
        self.no_line_count = 0

        # Buffer para suavizado
        self.error_buffer = []
        self.buffer_size = 3

        # Suscripción y publicador
        self.image_sub = None
        self.cmd_vel_pub = None

        # Flag para saber si el comportamiento está activo
        self.is_active = False

        # Aquí guardamos el último frame recibido
        self.latest_frame = None

    def setup(self, **kwargs):
        """
        Se llama una sola vez al configurar el árbol. Creamos suscripción y publicador.
        """
        if self.node:
            # Crear publicador para cmd_vel
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

            # Crear suscripción al tópico de la cámara
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
        También abrimos las ventanas de debug de OpenCV.
        """
        if self.node:
            self.node.get_logger().info("🚀 FollowLineSlow: iniciando seguimiento de línea lento")

        # Activar procesamiento
        self.is_active = True

        # Resetear variables de PID y contador de cuadros
        self.last_error = 0.0
        self.smoothed_error = 0.0
        self.integral_error = 0.0
        self.error_buffer.clear()
        self.no_line_count = 0
        self.frame_count = 0

        # Asegurarse de que las ventanas existan (solo una vez se crean)
        cv2.namedWindow('Debug - Original', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Debug - Procesado', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Debug - Original', 400, 300)
        cv2.resizeWindow('Debug - Procesado', 400, 300)

    def update(self):
        """
        Aquí procesamos el último frame recibido y aplicamos la lógica de seguimiento de línea lento.
        Mostramos las imágenes de debug y publicamos comandos Twist.
        """
        # Si no hemos recibido aún ningún frame, devolvemos RUNNING
        if self.latest_frame is None:
            return py_trees.common.Status.RUNNING

        # Copiar el último frame para no modificar la referencia
        try:
            image = self.latest_frame.copy()
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"FollowLineSlow: error copiando frame: {e}")
            return py_trees.common.Status.RUNNING

        # Convertir el frame BGR y recortar ROI
        height, width = image.shape[:2]
        roi_h = min(self.roi_height, height)
        roi_w = min(self.roi_width, width)

        start_y = max(0, height - roi_h)
        center_x = width // 2
        half_w = roi_w // 2
        start_x = max(0, center_x - half_w)
        end_x = min(width, center_x + half_w)

        # Validar ROI
        if (end_x - start_x) < 20 or (height - start_y) < 5:
            self._stop_robot()
            return py_trees.common.Status.RUNNING

        roi = image[start_y:height, start_x:end_x]

        # Preprocesar ROI (gris, blur, umbral adaptativo, morfología)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 1.0)

        try:
            binary = cv2.adaptiveThreshold(
                blurred, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                15, 8
            )
        except cv2.error:
            _, binary = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY_INV)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # Encontrar contornos y centroid de la línea
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > max(15, binary.shape[0] * binary.shape[1] * 0.008)]
        centroid = None
        if valid:
            M = cv2.moments(max(valid, key=cv2.contourArea))
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if 0 <= cx < binary.shape[1] and 0 <= cy < binary.shape[0]:
                    centroid = (cx, cy)

        twist = Twist()
        if centroid is not None:
            # Línea detectada: PID para velocidad lenta
            self.no_line_count = 0
            self.last_centroid = centroid

            error = centroid[0] - (binary.shape[1] // 2)
            normalized = error / float(binary.shape[1] // 2)

            # Calcular suavizado y PID
            self.error_buffer.append(normalized)
            if len(self.error_buffer) > self.buffer_size:
                self.error_buffer.pop(0)
            weights = np.linspace(0.7, 1.0, len(self.error_buffer))
            weighted_avg = np.average(self.error_buffer, weights=weights)
            self.smoothed_error = (1 - self.smoothing) * weighted_avg + self.smoothing * self.smoothed_error

            prop = self.kp * self.smoothed_error
            self.integral_error = np.clip(self.integral_error + self.smoothed_error, -self.max_integral, self.max_integral)
            integ = self.ki * self.integral_error
            deriv = self.kd * (self.smoothed_error - self.last_error)
            control = np.clip(prop + integ + deriv, -self.max_angular, self.max_angular)
            self.last_error = self.smoothed_error

            # Ajustar velocidades
            speed_factor = 1.0 - min(0.6, abs(self.smoothed_error) * 1.2)
            twist.linear.x = self.linear_speed * speed_factor
            twist.angular.z = -control

            # Mostrar debug cada 3 frames
            self.frame_count += 1
            if self.frame_count % 3 == 0:
                debug_roi = roi.copy()
                cv2.circle(debug_roi, centroid, 5, (0, 255, 0), -1)
                cv2.line(debug_roi,
                         (binary.shape[1] // 2, 0),
                         (binary.shape[1] // 2, binary.shape[0]),
                         (255, 0, 0), 2)
                cv2.putText(debug_roi, 'SLOW MODE', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                cv2.imshow('Debug - Original', debug_roi)
                cv2.imshow('Debug - Procesado', cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR))
                cv2.waitKey(1)

        else:
            # Si no se detecta línea, intentamos seguir última dirección o detener
            self.no_line_count += 1
            if self.no_line_count < 8 and self.last_centroid is not None:
                last_error = (self.last_centroid[0] - (binary.shape[1] // 2)) / float((binary.shape[1] // 2))
                ang = self.kp * last_error * 0.6
                twist.linear.x = self.linear_speed * 0.4
                twist.angular.z = -ang
            else:
                self._stop_robot()
                self.smoothed_error = 0.0
                self.integral_error = 0.0
                self.error_buffer.clear()

        # Publicar comando de velocidad
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Al salir de este comportamiento, desactivamos flag y detenemos el robot.
        No destruimos la suscripción para poder reutilizarla si volvemos a entrar.
        """
        self.is_active = False
        self._stop_robot()
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

    def _stop_robot(self):
        """
        Publicamos Twist(0,0) para detener el robot.
        """
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist())
