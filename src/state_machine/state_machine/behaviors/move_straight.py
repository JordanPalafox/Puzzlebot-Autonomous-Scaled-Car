# move_straight.py
import py_trees
from geometry_msgs.msg import Twist
import time

class MoveStraight(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveStraight", node=None, duration=4.0, linear_speed=0.10, angular_correction=0.08):
        super().__init__(name)
        self.node = node
        self.duration = duration  # Duración en segundos
        self.linear_speed = linear_speed  # Velocidad lineal
        self.angular_correction = angular_correction  # Corrección angular para compensar deriva (negativo = izquierda)
        
        # Estados internos
        self.start_time = None
        self.cmd_vel_pub = None
        self.is_active = False

    def setup(self, **kwargs):
        """Configurar el publicador de velocidad"""
        if self.node:
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.node.get_logger().info("🚀 MoveStraight: publicador cmd_vel creado")
        return True

    def initialise(self):
        """Inicializar el comportamiento"""
        if self.node:
            self.node.get_logger().info(f"🚀 MoveStraight: iniciando movimiento recto por {self.duration}s con corrección angular {self.angular_correction}")
        
        # Marcar tiempo de inicio
        self.start_time = time.time()
        self.is_active = True

    def update(self):
        """Actualizar el comportamiento"""
        if not self.is_active or self.start_time is None:
            return py_trees.common.Status.RUNNING

        # Calcular tiempo transcurrido
        elapsed_time = time.time() - self.start_time
        
        # Verificar si ya terminó la duración
        if elapsed_time >= self.duration:
            if self.node:
                self.node.get_logger().info(f"✅ MoveStraight: completado después de {elapsed_time:.1f}s")
            
            # Detener el robot
            self._stop_robot()
            return py_trees.common.Status.SUCCESS
        
        # Continuar avanzando recto con corrección angular
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_correction  # Corrección para compensar deriva hacia la derecha
        
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)
        
        # Mostrar progreso cada segundo
        if int(elapsed_time) != int(elapsed_time - 0.1):  # Aproximadamente cada segundo
            remaining = self.duration - elapsed_time
            if self.node:
                self.node.get_logger().debug(f"MoveStraight: {remaining:.1f}s restantes")
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Terminar el comportamiento"""
        if self.node:
            self.node.get_logger().info(f"🔚 MoveStraight: terminando con estado {new_status}")
        
        # Siempre detener el robot al terminar
        self._stop_robot()
        self.is_active = False
        self.start_time = None

    def _stop_robot(self):
        """Detener el robot"""
        if self.cmd_vel_pub:
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_twist)

    def set_duration(self, duration):
        """Cambiar la duración del movimiento"""
        self.duration = duration
        if self.node:
            self.node.get_logger().info(f"MoveStraight: duración cambiada a {duration}s")

    def set_speed(self, speed):
        """Cambiar la velocidad del movimiento"""
        self.linear_speed = speed
        if self.node:
            self.node.get_logger().info(f"MoveStraight: velocidad cambiada a {speed}m/s")

    def set_angular_correction(self, correction):
        """Cambiar la corrección angular para compensar deriva"""
        self.angular_correction = correction
        if self.node:
            self.node.get_logger().info(f"MoveStraight: corrección angular cambiada a {correction}rad/s") 