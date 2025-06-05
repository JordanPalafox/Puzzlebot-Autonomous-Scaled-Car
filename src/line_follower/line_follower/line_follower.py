#!/usr/bin/env python3
"""
Seguidor de línea robusto para PuzzleBot (ROS 2 + OpenCV)
Versión simplificada y estable - sin oscilaciones
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import math

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Parámetros simplificados - solo los esenciales
        self.declare_parameter('roi_height', 60, 
                             ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('roi_width', 200,
                             ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('threshold', 70,
                             ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('kp', 0.2,
                             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('ki', 0.05,
                             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('kd', 0.2,
                             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('linear_speed', 0.08,
                             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('max_angular', 0.18,
                             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('smoothing', 0.7,
                             ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('debug', True,
                             ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        self.update_parameters()
        self.create_timer(1.0, self.update_parameters)

        # Configuración ROS
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage, '/video_source/compressed',
            self.image_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables de estado simplificadas
        self.last_error = 0.0
        self.smoothed_error = 0.0
        self.integral_error = 0.0
        self.max_integral = 1.0  # Límite anti-windup
        self.last_centroid = None
        self.frame_count = 0
        self.no_line_count = 0
        
        # Buffer para suavizado
        self.error_buffer = []
        self.buffer_size = 5

        if self.debug:
            cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Procesado', cv2.WINDOW_NORMAL)

        self.get_logger().info('🤖 Seguidor de línea robusto iniciado')

    def update_parameters(self):
        """Actualiza parámetros dinámicamente"""
        self.roi_height = max(15, self.get_parameter('roi_height').value)  # Mínimo más bajo pero seguro
        self.roi_width = max(80, self.get_parameter('roi_width').value)   # Mínimo más bajo pero seguro
        self.threshold = max(30, min(150, self.get_parameter('threshold').value))
        self.kp = max(0.1, self.get_parameter('kp').value)
        self.ki = max(0.0, self.get_parameter('ki').value)
        self.kd = max(0.0, self.get_parameter('kd').value)
        self.linear_speed = max(0.05, self.get_parameter('linear_speed').value)
        self.max_angular = max(0.3, self.get_parameter('max_angular').value)
        self.smoothing = max(0.3, min(0.9, self.get_parameter('smoothing').value))
        self.debug = self.get_parameter('debug').value

    def preprocess_image(self, image):
        """Preprocesamiento robusto de la imagen con validaciones anti-crash"""
        height, width = image.shape[:2]
        
        # Validación básica de la imagen
        if height < 10 or width < 10:
            self.get_logger().warning(f'⚠️ Imagen muy pequeña: {width}x{height}')
            return None, None, 0
        
        # ROI en la parte inferior de la imagen con validaciones
        roi_height = min(self.roi_height, height)  # No exceder altura de imagen
        roi_width = min(self.roi_width, width)     # No exceder ancho de imagen
        
        start_y = max(0, height - roi_height)
        center_x = width // 2
        half_width = roi_width // 2
        start_x = max(0, center_x - half_width)
        end_x = min(width, center_x + half_width)
        
        # Validar que el ROI tenga tamaño mínimo viable
        if (end_x - start_x) < 20 or (height - start_y) < 5:
            self.get_logger().warning(f'⚠️ ROI demasiado pequeño: {end_x-start_x}x{height-start_y}')
            return None, None, 0
        
        roi = image[start_y:height, start_x:end_x]
        
        # Validar ROI extraído
        if roi.size == 0:
            self.get_logger().warning('⚠️ ROI vacío')
            return None, None, 0
            
        # Conversión a escala de grises
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Suavizado ligero
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.0)
        
        # Umbralización adaptativa para robustez
        try:
            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 21, 10)
        except cv2.error as e:
            self.get_logger().warning(f'⚠️ Error en threshold: {e}')
            # Fallback a threshold simple
            _, binary = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY_INV)
        
        # Filtro morfológico mínimo
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        return roi, binary, start_x

    def find_line_centroid(self, binary_image):
        """Encuentra el centroide de la línea de forma robusta con protecciones"""
        if binary_image is None or binary_image.size == 0:
            return None
            
        # Buscar contornos con manejo de errores
        try:
            contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except cv2.error:
            return None
        
        if not contours:
            return None
            
        # Filtrar contornos por área mínima (adaptada al tamaño del ROI)
        min_area = max(20, binary_image.shape[0] * binary_image.shape[1] * 0.01)  # 1% del ROI
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
        
        if not valid_contours:
            return None
            
        # Encontrar el contorno más grande (probablemente la línea)
        largest_contour = max(valid_contours, key=cv2.contourArea)
        
        # Calcular centroide usando momentos con validación
        try:
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return None
                
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Validar que el centroide esté dentro del ROI
            if 0 <= cx < binary_image.shape[1] and 0 <= cy < binary_image.shape[0]:
                return (cx, cy)
            else:
                return None
                
        except (ZeroDivisionError, cv2.error):
            return None

    def smooth_error(self, error):
        """Suavizado robusto del error"""
        # Agregar al buffer
        self.error_buffer.append(error)
        if len(self.error_buffer) > self.buffer_size:
            self.error_buffer.pop(0)
        
        # Promedio ponderado con más peso a valores recientes
        weights = np.linspace(0.5, 1.0, len(self.error_buffer))
        weighted_avg = np.average(self.error_buffer, weights=weights)
        
        # Suavizado exponencial
        self.smoothed_error = (1 - self.smoothing) * weighted_avg + self.smoothing * self.smoothed_error
        
        return self.smoothed_error

    def calculate_control(self, error):
        """Control PID completo y estable"""
        # Suavizar el error
        smooth_err = self.smooth_error(error)
        
        # Término Proporcional
        proportional = self.kp * smooth_err
        
        # Término Integral con anti-windup
        self.integral_error += smooth_err
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        integral = self.ki * self.integral_error
        
        # Término Derivativo
        derivative = self.kd * (smooth_err - self.last_error)
        
        # Control PID completo
        control_signal = proportional + integral + derivative
        
        # Limitar la salida para evitar oscilaciones
        control_signal = np.clip(control_signal, -self.max_angular, self.max_angular)
        
        # Guardar para la siguiente iteración
        self.last_error = smooth_err
        
        return control_signal

    def image_callback(self, msg):
        """Callback principal optimizado"""
        try:
            # Decodificar imagen
            image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.frame_count += 1
            
            # Preprocesar con validación
            roi, binary, roi_start_x = self.preprocess_image(image)
            
            # Validar que el preprocesamiento fue exitoso
            if roi is None or binary is None:
                self.get_logger().warning('⚠️ Preprocesamiento falló, deteniendo robot')
                self.cmd_vel_pub.publish(Twist())
                return
                
            roi_center = roi.shape[1] // 2
            
            # Validar división por cero
            if roi_center == 0:
                self.get_logger().warning('⚠️ ROI center es 0, deteniendo robot')
                self.cmd_vel_pub.publish(Twist())
                return
            
            # Encontrar línea
            centroid = self.find_line_centroid(binary)
            
            twist = Twist()
            
            if centroid is not None:
                # Línea detectada
                self.no_line_count = 0
                self.last_centroid = centroid
                
                # Calcular error (distancia del centro)
                error = centroid[0] - roi_center
                
                # Normalizar error
                normalized_error = error / roi_center
                
                # Calcular control
                angular_velocity = self.calculate_control(normalized_error)
                
                # Velocidad lineal adaptativa
                error_magnitude = abs(normalized_error)
                speed_factor = 1.0 - min(0.8, error_magnitude * 1.5)
                linear_velocity = self.linear_speed * speed_factor
                
                # Publicar comandos
                twist.linear.x = linear_velocity
                twist.angular.z = -angular_velocity  # Negativo para corrección correcta
                
                if self.debug and self.frame_count % 3 == 0:  # Reducir frecuencia de debug
                    debug_roi = roi.copy()
                    cv2.circle(debug_roi, centroid, 5, (0, 255, 0), -1)
                    cv2.line(debug_roi, (roi_center, 0), (roi_center, roi.shape[0]), (255, 0, 0), 2)
                    cv2.imshow('Original', debug_roi)
                    cv2.imshow('Procesado', cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR))
                    cv2.waitKey(1)
                    
            else:
                # No se detectó línea
                self.no_line_count += 1
                
                if self.no_line_count < 15 and self.last_centroid is not None and roi_center > 0:
                    # Usar último error conocido por poco tiempo (con protección división por cero)
                    last_error = (self.last_centroid[0] - roi_center) / roi_center
                    angular_velocity = self.calculate_control(last_error) * 0.5
                    twist.linear.x = self.linear_speed * 0.3
                    twist.angular.z = -angular_velocity
                else:
                                         # Parar completamente
                     twist.linear.x = 0.0
                     twist.angular.z = 0.0
                     self.smoothed_error = 0.0
                     self.integral_error = 0.0  # Reset integral
                     self.error_buffer.clear()
            
            # Publicar comando
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Error en procesamiento: {e}')
            self.cmd_vel_pub.publish(Twist())  # Parar en caso de error

    def destroy_node(self):
        """Limpieza al cerrar"""
        self.cmd_vel_pub.publish(Twist())  # Parar robot
        if self.debug:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Deteniendo seguidor de línea...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()