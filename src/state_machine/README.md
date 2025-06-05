# State Machine para Puzzlebot

Este paquete implementa una máquina de estados para el robot Puzzlebot utilizando la biblioteca py_trees para ROS 2. La máquina de estados permite controlar diferentes comportamientos del robot como seguir líneas, girar y detenerse.

## Descripción

La máquina de estados incluye los siguientes comportamientos:
- **idle**: Estado de reposo (comportamiento por defecto)
- **followLineFast**: Seguimiento de línea a velocidad rápida
- **followLineSlow**: Seguimiento de línea a velocidad lenta
- **turnRight**: Giro hacia la derecha
- **turnLeft**: Giro hacia la izquierda
- **stop**: Detener el robot

## Dependencias

Este paquete requiere las siguientes dependencias:
- ROS 2 (Humble/Iron)
- rclpy
- py_trees
- py_trees_ros
- python3-numpy
- std_msgs

## Instalación

### 1. Clonar el repositorio
```bash
cd ~/ros2_ws/src
git clone <tu-repositorio>
```

### 2. Instalar dependencias
```bash
# Instalar dependencias de ROS
sudo apt update
sudo apt install ros-$ROS_DISTRO-py-trees ros-$ROS_DISTRO-py-trees-ros

# Instalar dependencias de Python
pip3 install py_trees
```

### 3. Compilar el paquete
```bash
cd ~/ros2_ws
colcon build --packages-select state_machine
source install/setup.bash
```

## Uso

### 1. Ejecutar la máquina de estados
En una terminal, ejecuta el nodo principal de la máquina de estados:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run state_machine state_machine
```

### 2. Enviar comandos a la máquina de estados
En otra terminal, puedes enviar comandos utilizando el script auxiliar:
```bash
source ~/ros2_ws/install/setup.bash

# Ejemplos de comandos:
ros2 run state_machine send_state_command idle
ros2 run state_machine send_state_command followLineFast
ros2 run state_machine send_state_command followLineSlow
ros2 run state_machine send_state_command turnRight
ros2 run state_machine send_state_command turnLeft
ros2 run state_machine send_state_command stop
```

### 3. Enviar comandos directamente con ros2 topic
También puedes enviar comandos directamente usando ros2 topic:
```bash
# Ejemplo para enviar comando de seguir línea rápido
ros2 topic pub /state_command std_msgs/msg/String "data: 'followLineFast'"

# Ejemplo para detener el robot
ros2 topic pub /state_command std_msgs/msg/String "data: 'stop'"
```

## Estructura del paquete

```
state_machine/
├── package.xml              # Metadatos del paquete ROS
├── setup.py                 # Configuración de instalación
├── setup.cfg                # Configuración adicional
├── README.md                # Este archivo
├── resource/
│   └── state_machine        # Archivo marcador de ament
└── state_machine/
    ├── __init__.py
    ├── state_machine.py     # Nodo principal de la máquina de estados
    ├── send_state_command.py # Script auxiliar para enviar comandos
    └── behaviors/           # Carpeta con los comportamientos
        ├── __init__.py
        ├── follow_line_fast.py
        ├── follow_line_slow.py
        ├── turn_left.py
        ├── turn_right.py
        └── stop.py
```

## Topics ROS

### Suscripciones
- `/state_command` (std_msgs/String): Recibe comandos para cambiar el estado de la máquina

### Publicaciones
Los comportamientos individuales publican en sus respectivos topics según su implementación:
- `/cmd_vel` (geometry_msgs/Twist): Comandos de velocidad para el robot
- Otros topics específicos según el comportamiento

## Comandos disponibles

| Comando | Descripción |
|---------|-------------|
| `idle` | Estado de reposo (por defecto) |
| `followLineFast` | Seguir línea a velocidad rápida |
| `followLineSlow` | Seguir línea a velocidad lenta |
| `turnRight` | Girar hacia la derecha |
| `turnLeft` | Girar hacia la izquierda |
| `stop` | Detener el robot |

## Solución de problemas

### Error: "package directory does not exist"
- Asegúrate de que el nombre del paquete en `setup.py` coincide con el nombre de la carpeta
- Verifica que la estructura de directorios sea correcta

### Error: "No module named 'py_trees'"
- Instala py_trees: `pip3 install py_trees`
- O usando apt: `sudo apt install ros-$ROS_DISTRO-py-trees`

### La máquina de estados no responde a comandos
- Verifica que ambos nodos estén ejecutándose
- Comprueba que los topics estén activos: `ros2 topic list`
- Revisa los logs para errores: `ros2 node info /state_machine_node`

## Desarrollo

Para añadir nuevos comportamientos:
1. Crea un nuevo archivo en `state_machine/behaviors/`
2. Implementa una clase que herede de `py_trees.behaviour.Behaviour`
3. Añade el comportamiento a la máquina de estados en `state_machine.py`
4. Actualiza la documentación

## Autor

Jordan Palafox (a00835705@tec.mx)

## Licencia

Apache License 2.0
