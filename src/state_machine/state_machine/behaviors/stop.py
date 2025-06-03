#!/usr/bin/env python3
"""
Behavior para detener el robot.
"""

import py_trees
from geometry_msgs.msg import Twist

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name="Stop", node=None):
        super().__init__(name)
        self.node = node
        self.cmd_vel_pub = None
    
    def setup(self, **kwargs):
        if self.node:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("Stop behavior configurado")
        return True
    
    def initialise(self):
        if self.node:
            self.node.get_logger().info("Deteniendo robot")
        self.stop_robot()
    
    def update(self):
        # Continuamente envía comando de parada
        self.stop_robot()
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.stop_robot()
        if self.node:
            self.node.get_logger().info("Behavior de parada terminado")
    
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