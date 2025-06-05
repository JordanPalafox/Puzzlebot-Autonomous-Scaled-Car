#!/usr/bin/env python3
"""
Behavior para girar a la derecha usando odometría.
"""

import py_trees
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from rclpy import qos
import time

class TurnRight(py_trees.behaviour.Behaviour):
    def __init__(self, name="TurnRight", node=None, angle=np.pi/2, angular_speed=0.5, linear_speed=0.06, use_odometry=True, duration_fallback=3.0):
        super().__init__(name)
        self.node = node
        self.target_angle = angle  # 90 grados por defecto (π/2 radianes)
        self.angular_speed = angular_speed  # Velocidad angular aumentada
        self.linear_speed = linear_speed  # Velocidad lineal para avanzar mientras gira
        self.use_odometry = use_odometry  # Si usar odometría o tiempo
        self.duration_fallback = duration_fallback  # Duración fallback aumentada
        self.cmd_vel_pub = None
        self.odom_sub = None
        
        # Control variables
        self.current_pose = None
        self.initial_yaw = None
        self.target_yaw = None
        self.start_time = None
        self.has_odom_data = False
        
        # PID parameters mejorados para control angular
        self.Kp_angular = 2.0  # Proporción aumentada
        self.Ki_angular = 0.1   # Integral ajustado
        self.Kd_angular = 0.5   # Derivativo aumentado
        
        # Error accumulators
        self.integral_angular = 0.0
        self.prev_error_angular = 0.0
        self.last_time = None
        
        # Threshold más preciso para considerar completado el giro
        self.angle_threshold = 0.02  # ~4.5 grados, un poco más tolerante
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convertir quaternion a ángulos de Euler (roll, pitch, yaw).
        Implementación manual para evitar dependencias problemáticas.
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
        
    def setup(self, **kwargs):
        if self.node:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            if self.use_odometry:
                self.odom_sub = self.node.create_subscription(
                    Odometry,
                    '/odom',
                    self.odom_callback,
                    qos.qos_profile_sensor_data
                )
                self.node.get_logger().info("TurnRight behavior configurado con odometría")
            else:
                self.node.get_logger().info("TurnRight behavior configurado con tiempo fijo")
        return True
    
    def odom_callback(self, msg):
        """Almacenar la pose actual del robot desde la odometría"""
        self.current_pose = msg.pose.pose
        if not self.has_odom_data:
            self.has_odom_data = True
            if self.node:
                self.node.get_logger().info("¡Datos de odometría recibidos!")
    
    def initialise(self):
        self.integral_angular = 0.0
        self.prev_error_angular = 0.0
        self.last_time = time.time()
        self.start_time = time.time()
        
        if self.node:
            self.node.get_logger().info(f"Iniciando TurnRight - Ángulo objetivo: {np.degrees(self.target_angle):.1f}° - Odometría: {self.use_odometry}")
            self.node.get_logger().info(f"Velocidades: angular={self.angular_speed:.2f} rad/s, lineal={self.linear_speed:.2f} m/s")
        
        if self.use_odometry and self.current_pose is not None:
            # Obtener la orientación actual como ángulos de Euler
            orientation = self.current_pose.orientation
            _, _, current_yaw = self.euler_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            
            self.initial_yaw = current_yaw
            self.target_yaw = self.wrap_to_pi(current_yaw - self.target_angle)  # Giro a la derecha (negativo)
            
            if self.node:
                self.node.get_logger().info(f"Ángulo inicial: {np.degrees(current_yaw):.1f}°")
                self.node.get_logger().info(f"Ángulo objetivo: {np.degrees(self.target_yaw):.1f}°")
                self.node.get_logger().info(f"Diferencia a girar: {np.degrees(self.target_angle):.1f}°")
        else:
            if self.node:
                self.node.get_logger().info(f"Modo tiempo fijo - {self.duration_fallback}s")
    
    def update(self):
        current_time = time.time()
        
        # Crear comando de velocidad básico - SIEMPRE avanza
        twist = Twist()
        twist.linear.x = self.linear_speed  # Avanzar mientras gira
        
        # Si tenemos odometría y datos disponibles, usar control preciso
        if self.use_odometry and self.has_odom_data and self.current_pose is not None and self.initial_yaw is not None:
            
            # Obtener orientación actual
            orientation = self.current_pose.orientation
            _, _, current_yaw = self.euler_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            
            # Calcular error angular
            angle_error = self.wrap_to_pi(self.target_yaw - current_yaw)
            total_turned = self.wrap_to_pi(current_yaw - self.initial_yaw)
            
            if self.node and (current_time - self.start_time) % 0.8 < 0.1:  # Log cada 0.8 segundos
                self.node.get_logger().info(f"Actual: {np.degrees(current_yaw):.1f}°, Girado: {np.degrees(-total_turned):.1f}°, Error: {np.degrees(angle_error):.1f}°")
            
            # Control PID para movimiento angular
            dt = current_time - self.last_time if self.last_time else 0.1
            
            # Limitar el integral para evitar windup
            self.integral_angular += angle_error * dt
            self.integral_angular = np.clip(self.integral_angular, -1.0, 1.0)
            
            derivative = (angle_error - self.prev_error_angular) / dt if dt > 0 else 0.0
            
            angular_vel = (
                self.Kp_angular * angle_error +
                self.Ki_angular * self.integral_angular +
                self.Kd_angular * derivative
            )
            
            # Limitar velocidad angular con mínimo para asegurar movimiento
            angular_vel = np.clip(angular_vel, -self.angular_speed, self.angular_speed)
            
            # Asegurar un mínimo de movimiento si el error es significativo
            if abs(angle_error) > self.angle_threshold:
                min_angular = 0.1  # Velocidad mínima para asegurar movimiento
                if angular_vel > 0:
                    angular_vel = max(angular_vel, min_angular)
                else:
                    angular_vel = min(angular_vel, -min_angular)
            
            twist.angular.z = angular_vel
            
            # Actualizar variables para siguiente iteración
            self.prev_error_angular = angle_error
            self.last_time = current_time
            
            # Verificar si se alcanzó el objetivo
            if abs(angle_error) < self.angle_threshold:
                self.stop_robot()
                if self.node:
                    self.node.get_logger().info(f"¡Giro completado! Girado total: {np.degrees(-total_turned):.1f}°")
                return py_trees.common.Status.SUCCESS
        
        else:
            # Modo fallback: usar tiempo fijo
            elapsed_time = current_time - self.start_time
            
            if elapsed_time < self.duration_fallback:
                # Girar a la derecha con velocidad fija Y avanzar
                twist.angular.z = -self.angular_speed  # Negativo para girar a la derecha
                
                if self.node and elapsed_time % 0.5 < 0.1:  # Log cada medio segundo
                    remaining = self.duration_fallback - elapsed_time
                    degrees_per_sec = np.degrees(self.angular_speed)
                    expected_turn = degrees_per_sec * elapsed_time
                    self.node.get_logger().info(f"Girando {remaining:.1f}s restantes - ~{expected_turn:.1f}° girados")
            else:
                self.stop_robot()
                if self.node:
                    expected_total = np.degrees(self.angular_speed * self.duration_fallback)
                    self.node.get_logger().info(f"Giro por tiempo completado - ~{expected_total:.1f}° esperados")
                return py_trees.common.Status.SUCCESS
        
        # Publicar comando
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("Terminando giro a la derecha")
    
    def stop_robot(self):
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist())
    
    def wrap_to_pi(self, theta):
        """Envolver ángulo al rango [-π, π]"""
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi 