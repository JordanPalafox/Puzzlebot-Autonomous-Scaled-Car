#!/usr/bin/env python3
"""
Behavior para mantener el robot en estado idle (parado).
"""

import py_trees
from geometry_msgs.msg import Twist

class Idle(py_trees.behaviour.Behaviour):
    def __init__(self, name="Idle", node=None):
        super().__init__(name)
        self.node = node
        self.cmd_vel_pub = None
    
    def setup(self, **kwargs):
        if self.node:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("😴 Idle behavior configurado")
        return True
    
    def initialise(self):
        if self.node:
            self.node.get_logger().info("😴 Robot en estado IDLE - parado")
        self.stop_robot()
    
    def update(self):
        # Continuamente envía comando de parada para mantener idle
        self.stop_robot()
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("😴 Saliendo de estado idle")
    
    def stop_robot(self):
        if self.cmd_vel_pub:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist) 