#!/usr/bin/env python3
"""
Behavior para girar a la derecha.
"""

import py_trees
from geometry_msgs.msg import Twist
import time

class TurnRight(py_trees.behaviour.Behaviour):
    def __init__(self, name="TurnRight", node=None, duration=2.0, angular_speed=0.3):
        super().__init__(name)
        self.node = node
        self.duration = duration
        self.angular_speed = angular_speed
        self.cmd_vel_pub = None
        self.start_time = None
    
    def setup(self, **kwargs):
        if self.node:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("TurnRight behavior configurado")
        return True
    
    def initialise(self):
        self.start_time = time.time()
        if self.node:
            self.node.get_logger().info(f"Iniciando giro a la derecha por {self.duration} segundos")
    
    def update(self):
        if self.start_time is None:
            return py_trees.common.Status.FAILURE
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < self.duration:
            # Girar a la derecha (velocidad angular negativa)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
            if self.cmd_vel_pub:
                self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING
        else:
            # Detener el robot
            self.stop_robot()
            if self.node:
                self.node.get_logger().info("Giro a la derecha completado")
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("Terminando giro a la derecha")
    
    def stop_robot(self):
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist()) 