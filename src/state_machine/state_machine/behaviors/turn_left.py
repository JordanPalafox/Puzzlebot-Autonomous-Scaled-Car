#!/usr/bin/env python3
"""
Behavior para girar a la izquierda.
"""

import py_trees
from geometry_msgs.msg import Twist
import time

class TurnLeft(py_trees.behaviour.Behaviour):
    def __init__(self, name="TurnLeft", node=None, duration=2.0, angular_speed=0.3):
        super().__init__(name)
        self.node = node
        self.duration = duration
        self.angular_speed = angular_speed
        self.cmd_vel_pub = None
        self.start_time = None
    
    def setup(self, **kwargs):
        if self.node:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("TurnLeft behavior configurado")
        return True
    
    def initialise(self):
        self.start_time = time.time()
        if self.node:
            self.node.get_logger().info(f"Iniciando giro a la izquierda por {self.duration} segundos")
    
    def update(self):
        if self.start_time is None:
            return py_trees.common.Status.FAILURE
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < self.duration:
            # Girar a la izquierda (velocidad angular positiva)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            if self.cmd_vel_pub:
                self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Detener el robot
            self.stop_robot()
            if self.node:
                self.node.get_logger().info("Giro a la izquierda completado")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("Terminando giro a la izquierda")
    
    def stop_robot(self):
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist()) 