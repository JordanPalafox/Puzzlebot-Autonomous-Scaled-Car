import time
import rclpy
from rclpy.node import Node
import py_trees
import threading
import cv2
from std_msgs.msg import String
from state_machine.behaviors.follow_line_fast import FollowLineFast
from state_machine.behaviors.follow_line_slow import FollowLineSlow
from state_machine.behaviors.turn_right import TurnRight
from state_machine.behaviors.turn_left import TurnLeft
from state_machine.behaviors.stop import Stop
from state_machine.behaviors.idle import Idle

# Variable global para almacenar el comando actual
current_state_command = "idle"
last_completed_state = None

# Comportamiento personalizado que selecciona directamente el comportamiento según el estado
class StateMachineRoot(py_trees.behaviour.Behaviour):
    def __init__(self, name="StateMachine"):
        super().__init__(name)
        self.state_behaviors = {}
        self.current_behavior = None
        self.node = None
        self.behaviors_setup_done = False

    def setup_with_node(self, node):
        self.node = node
        # No creamos ventanas aquí; cada comportamiento lo hará en su initialise() si lo necesita

    def add_state(self, state_name, behavior):
        self.state_behaviors[state_name] = behavior

    def setup_all_behaviors(self):
        if self.behaviors_setup_done:
            return

        if self.node:
            self.node.get_logger().info("Configurando todos los comportamientos...")

        for state_name, behavior in self.state_behaviors.items():
            try:
                # Todos los behaviors tienen setup(timeout_sec) o setup()
                import inspect
                sig = inspect.signature(behavior.setup)
                if 'timeout_sec' in sig.parameters:
                    behavior.setup(timeout_sec=5.0)
                else:
                    behavior.setup()
                if self.node:
                    self.node.get_logger().info(f"✔ '{state_name}' configurado")
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Error al configurar '{state_name}': {e}")

        self.behaviors_setup_done = True
        if self.node:
            self.node.get_logger().info("✔ Todos los comportamientos configurados")

    def initialise(self):
        if self.node:
            self.node.get_logger().info(f"State machine inicializada, estado actual: {current_state_command}")

    def update(self):
        global current_state_command, last_completed_state

        # 1. Asegurarnos de configurar behaviors
        if not self.behaviors_setup_done:
            self.setup_all_behaviors()

        # 2. Si el estado no existe, esperamos
        if current_state_command not in self.state_behaviors:
            if self.node:
                self.node.get_logger().warn(f"Estado solicitado '{current_state_command}' no existe")
            return py_trees.common.Status.RUNNING

        # 3. Si cambiamos de comportamiento, hacemos terminate() del anterior e initialise() del nuevo
        desired_behavior = self.state_behaviors[current_state_command]
        if self.current_behavior is None or desired_behavior != self.current_behavior:
            # Finalizar comportamiento anterior
            if self.current_behavior is not None:
                try:
                    if self.node:
                        self.node.get_logger().info(f"🔄 Terminando '{self.current_behavior.name}'")
                    self.current_behavior.terminate(py_trees.common.Status.INVALID)
                except Exception as e:
                    if self.node:
                        self.node.get_logger().error(f"Error al terminar '{self.current_behavior.name}': {e}")

            # Cambiamos al nuevo
            self.current_behavior = desired_behavior
            try:
                if self.node:
                    self.node.get_logger().info(f"🔄 Entrando en '{self.current_behavior.name}'")
                self.current_behavior.initialise()
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Error al inicializar '{self.current_behavior.name}': {e}")
                return py_trees.common.Status.RUNNING

        # 4. Ejecutamos el comportamiento actual
        try:
            status = self.current_behavior.update()
            if status == py_trees.common.Status.SUCCESS:
                last_completed_state = current_state_command
                if self.node:
                    self.node.get_logger().info(f"✔ Estado '{current_state_command}' completado")
            return py_trees.common.Status.RUNNING
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error en update de '{self.current_behavior.name}': {e}")
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.current_behavior is not None:
            try:
                self.current_behavior.terminate(new_status)
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Error al terminar '{self.current_behavior.name}': {e}")


def state_command_callback(msg):
    global current_state_command, last_completed_state
    if current_state_command != msg.data:
        print(f"Cambiando a estado: {msg.data}")
        current_state_command = msg.data
        last_completed_state = None

def spin_ros(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = Node('state_machine_node')

    # Suscriptor de comandos de estado
    node.create_subscription(String, 'state_command', state_command_callback, 10)

    # Pequeña pausa para asegurar que ROS 2 esté listo
    time.sleep(1.0)

    idle = Idle(name="Idle", node=node)
    followLineFast = FollowLineFast(name="FollowLineFast", node=node)
    followLineSlow = FollowLineSlow(name="FollowLineSlow", node=node)
    turnRight = TurnRight(name="TurnRight", node=node)
    turnLeft = TurnLeft(name="TurnLeft", node=node)
    stop = Stop(name="Stop", node=node)

    # Crear el Behavior Tree
    root = StateMachineRoot()
    root.setup_with_node(node)
    root.add_state("idle", idle)
    root.add_state("followLineFast", followLineFast)
    root.add_state("followLineSlow", followLineSlow)
    root.add_state("turnRight", turnRight)
    root.add_state("turnLeft", turnLeft)
    root.add_state("stop", stop)

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup()

    # Spin de ROS en hilo aparte para atender callbacks
    spin_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    spin_thread.start()

    node.get_logger().info("✅ State machine arrancada")

    # Bucle principal para hacer tick() al Behavior Tree
    try:
        while rclpy.ok():
            tree.tick()
            time.sleep(0.05)  # Menos delay, para que update() de FollowLineFast refresque con más frecuencia
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🔴 Apagando state machine")
        # Al cerrar, asegurarse de destruir ventanas de OpenCV
        try:
            cv2.destroyAllWindows()
        except:
            pass
        rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()