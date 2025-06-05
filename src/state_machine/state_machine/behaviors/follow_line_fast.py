#!/usr/bin/env python3
"""
Behavior robusto para seguimiento de línea rápido usando OpenCV.
Basado en el código robusto del line_follower.py
"""

import cv2
import numpy as np
import py_trees
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

class FollowLineFast(py_trees.behaviour.Behaviour):
    def __init__(self, name="FollowLineFast", node=None):
        super().__init__(name)
        self.node = node
        self.bridge = CvBridge()
        
        # Parámetros para velocidad RÁPIDA
        self.roi_height = 30        # ROI más pequeño para respuesta rápida
        self.roi_width = 200        # ROI más estrecho
        self.threshold = 75
        self.kp = 0.2             # Más agresivo
        self.ki = 0.05             # Integral para eliminar error residual
        self.kd = 0.2             # Derivativo para suavizar
        self.linear_speed = 0.12   # Velocidad RÁPIDA
        self.max_angular = 0.18     # Giros más agresivos
        self.smoothing = 0.7       # Menos suavizado para respuesta rápida
        
        # Variables de estado simplificadas
        self.last_error = 0.0
        self.smoothed_error = 0.0
        self.integral_error = 0.0
        self.max_integral = 1.0
        self.last_centroid = None
        self.frame_count = 0
        self.no_line_count = 0
        
        # Buffer para suavizado
        self.error_buffer = []
        self.buffer_size = 3       # Buffer más pequeño para respuesta rápida
        
        # Suscripciones y publicadores
        self.image_sub = None
        self.cmd_vel_pub = None
    
    def setup(self, **kwargs):
        if self.node:
            # Solo crear el publicador en setup, NO el suscriptor
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("🚀 FollowLineFast behavior robusto configurado")
        return True
    
    def initialise(self):
        if self.node:
            self.node.get_logger().info("🚀 Iniciando seguimiento de línea RÁPIDO")
        # Reset de variables al inicializar
        self.last_error = 0.0
        self.smoothed_error = 0.0
        self.integral_error = 0.0
        self.error_buffer.clear()
        # Recrear suscriptor si no existe
        if not self.image_sub and self.node:
            self.image_sub = self.node.create_subscription(
                CompressedImage, '/video_source/compressed',
                self.image_callback, qos_profile_sensor_data)
    
    def update(self):
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.stop_robot()
        # Destruir suscriptor para evitar que siga ejecutándose
        if self.image_sub:
            self.node.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.node:
            self.node.get_logger().info("🛑 Terminando seguimiento de línea rápido - suscriptor destruido")

    def preprocess_image(self, image):
        """Preprocesamiento robusto optimizado para velocidad rápida"""
        height, width = image.shape[:2]
        
        # Validación básica
        if height < 10 or width < 10:
            return None, None, 0
        
        # ROI optimizado para velocidad rápida
        roi_height = min(self.roi_height, height)
        roi_width = min(self.roi_width, width)
        
        start_y = max(0, height - roi_height)
        center_x = width // 2
        half_width = roi_width // 2
        start_x = max(0, center_x - half_width)
        end_x = min(width, center_x + half_width)
        
        # Validar ROI mínimo
        if (end_x - start_x) < 20 or (height - start_y) < 5:
            return None, None, 0
        
        roi = image[start_y:height, start_x:end_x]
        
        if roi.size == 0:
            return None, None, 0
            
        # Procesamiento rápido y robusto
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 1.0)  # Menos blur para velocidad
        
        try:
            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 15, 8)  # Kernel más pequeño para velocidad
        except cv2.error:
            _, binary = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Filtro morfológico mínimo
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        return roi, binary, start_x

    def find_line_centroid(self, binary_image):
        """Encuentra centroide optimizado para velocidad rápida"""
        if binary_image is None or binary_image.size == 0:
            return None
            
        try:
            contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except cv2.error:
            return None
        
        if not contours:
            return None
            
        # Área mínima adaptativa más permisiva para velocidad
        min_area = max(15, binary_image.shape[0] * binary_image.shape[1] * 0.008)
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
        
        if not valid_contours:
            return None
            
        largest_contour = max(valid_contours, key=cv2.contourArea)
        
        try:
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return None
                
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            if 0 <= cx < binary_image.shape[1] and 0 <= cy < binary_image.shape[0]:
                return (cx, cy)
            else:
                return None
                
        except (ZeroDivisionError, cv2.error):
            return None

    def smooth_error(self, error):
        """Suavizado optimizado para respuesta rápida"""
        self.error_buffer.append(error)
        if len(self.error_buffer) > self.buffer_size:
            self.error_buffer.pop(0)
        
        # Menos suavizado para respuesta más rápida
        weights = np.linspace(0.7, 1.0, len(self.error_buffer))
        weighted_avg = np.average(self.error_buffer, weights=weights)
        
        self.smoothed_error = (1 - self.smoothing) * weighted_avg + self.smoothing * self.smoothed_error
        
        return self.smoothed_error

    def calculate_control(self, error):
        """Control PID optimizado para velocidad rápida"""
        smooth_err = self.smooth_error(error)
        
        # PID con ganancias para velocidad rápida
        proportional = self.kp * smooth_err
        
        self.integral_error += smooth_err
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        integral = self.ki * self.integral_error
        
        derivative = self.kd * (smooth_err - self.last_error)
        
        control_signal = proportional + integral + derivative
        control_signal = np.clip(control_signal, -self.max_angular, self.max_angular)
        
        self.last_error = smooth_err
        
        return control_signal

    def image_callback(self, msg):
        """Callback principal optimizado para velocidad rápida"""
        try:
            # Decodificar imagen
            image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.frame_count += 1
            
            # Preprocesar con validación
            roi, binary, roi_start_x = self.preprocess_image(image)
            
            if roi is None or binary is None:
                self.stop_robot()
                return
                
            roi_center = roi.shape[1] // 2
            
            if roi_center == 0:
                self.stop_robot()
                return
            
            # Encontrar línea
            centroid = self.find_line_centroid(binary)
            
            twist = Twist()
            
            if centroid is not None:
                # Línea detectada - control agresivo para velocidad rápida
                self.no_line_count = 0
                self.last_centroid = centroid
                
                error = centroid[0] - roi_center
                normalized_error = error / roi_center
                
                angular_velocity = self.calculate_control(normalized_error)
                
                # Velocidad lineal menos conservadora para modo rápido
                error_magnitude = abs(normalized_error)
                speed_factor = 1.0 - min(0.6, error_magnitude * 1.2)  # Menos reducción
                linear_velocity = self.linear_speed * speed_factor
                
                twist.linear.x = linear_velocity
                twist.angular.z = -angular_velocity
                
                # Debug visual (cada 3 frames para performance)
                if self.frame_count % 3 == 0:
                    debug_roi = roi.copy()
                    cv2.circle(debug_roi, centroid, 5, (0, 255, 0), -1)
                    cv2.line(debug_roi, (roi_center, 0), (roi_center, roi.shape[0]), (255, 0, 0), 2)
                    cv2.putText(debug_roi, 'FAST MODE', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.imshow('Debug - Original', debug_roi)
                    cv2.imshow('Debug - Procesado', cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR))
                    cv2.waitKey(1)
                    
            else:
                # No se detectó línea
                self.no_line_count += 1
                
                if self.no_line_count < 8 and self.last_centroid is not None and roi_center > 0:  # Menos tolerante para velocidad
                    last_error = (self.last_centroid[0] - roi_center) / roi_center
                    angular_velocity = self.calculate_control(last_error) * 0.6  # Más agresivo
                    twist.linear.x = self.linear_speed * 0.4
                    twist.angular.z = -angular_velocity
                else:
                    # Parar completamente
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.smoothed_error = 0.0
                    self.integral_error = 0.0
                    self.error_buffer.clear()
            
            # Publicar comando
            if self.cmd_vel_pub:
                self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Error en seguimiento rápido: {e}')
            self.stop_robot()

    def stop_robot(self):
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist()) 