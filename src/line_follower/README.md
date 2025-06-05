# Line Follower para PuzzleBot

Este paquete implementa un nodo de seguimiento de línea avanzado para el robot PuzzleBot utilizando OpenCV y ROS 2. El nodo procesa imágenes de cámara para detectar y seguir líneas utilizando algoritmos de visión por computadora y control PID.

## Características

- **Procesamiento de imagen avanzado**: Utiliza múltiples técnicas de procesamiento incluyendo umbralización adaptativa, detección de bordes y operaciones morfológicas
- **Control PID dual**: Implementa control PID separado para posición y ángulo con combinación ponderada
- **Parámetros dinámicos**: Todos los parámetros son configurables en tiempo real
- **Modo de recuperación**: Sistema inteligente de recuperación cuando se pierde la línea
- **Debug visual**: Ventanas de depuración para visualizar el procesamiento en tiempo real
- **Filtrado temporal**: Suavizado de detecciones para mayor estabilidad

## Dependencias

### Dependencias de ROS 2
- `rclpy`: Cliente Python para ROS 2
- `sensor_msgs`: Mensajes para datos de sensores (CompressedImage)
- `geometry_msgs`: Mensajes de geometría (Twist)
- `cv_bridge`: Conversión entre OpenCV y ROS
- `rcl_interfaces`: Interfaces de parámetros
- `std_msgs`: Mensajes estándar

### Dependencias de Python
- `opencv-python` (cv2): Procesamiento de imágenes
- `numpy`: Operaciones numéricas
- `math`: Funciones matemáticas

## Instalación

### 1. Instalar dependencias del sistema
```bash
# Instalar OpenCV y numpy
sudo apt update
sudo apt install python3-opencv python3-numpy

# Instalar dependencias de ROS 2
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-geometry-msgs
```

### 2. Compilar el paquete
```bash
cd ~/ros2_ws
colcon build --packages-select line_follower
source install/setup.bash
```

## Uso

### Ejecutar el nodo
```bash
source ~/ros2_ws/install/setup.bash
ros2 run line_follower line_follower
```

### Configurar parámetros en tiempo real
```bash
# Ajustar velocidad lineal
ros2 param set /line_follower linear_speed 0.1

# Ajustar gains PID de posición
ros2 param set /line_follower kp 0.4
ros2 param set /line_follower ki 0.01
ros2 param set /line_follower kd 0.05

# Habilitar/deshabilitar modo debug
ros2 param set /line_follower debug_mode true

# Ver todos los parámetros
ros2 param list /line_follower
```

## Parámetros configurables

### Parámetros de visión
- `vision_height` (int): Altura del ROI en píxeles (default: 10)
- `roi_center_width` (int): Ancho del ROI en píxeles (default: 140)
- `threshold_value` (int): Valor de umbralización (default: 80)
- `min_area` (int): Área mínima de componentes (default: 15)
- `max_area` (int): Área máxima de componentes (default: 1000)

### Parámetros de procesamiento
- `blur_kernel` (int): Tamaño del kernel de blur (default: 9)
- `morph_kernel` (int): Tamaño del kernel morfológico (default: 7)
- `erode_iterations` (int): Iteraciones de erosión (default: 3)
- `dilate_iterations` (int): Iteraciones de dilatación (default: 3)
- `use_edge_detection` (bool): Habilitar detección de bordes (default: true)

### Parámetros de control PID
- `kp` (double): Gain proporcional para posición (default: 0.3)
- `ki` (double): Gain integral para posición (default: 0.0)
- `kd` (double): Gain derivativo para posición (default: 0.0)
- `kp_angle` (double): Gain proporcional para ángulo (default: 0.08)
- `ki_angle` (double): Gain integral para ángulo (default: 0.0)
- `kd_angle` (double): Gain derivativo para ángulo (default: 0.0)

### Parámetros de control general
- `linear_speed` (double): Velocidad lineal base (default: 0.08)
- `max_angular_speed` (double): Velocidad angular máxima (default: 0.18)
- `angle_weight` (double): Peso del control angular vs posicional (default: 0.4)
- `target_angle` (double): Ángulo objetivo de la línea (default: 0.0)
- `memory_factor` (double): Factor de suavizado temporal (default: 0.7)
- `recovery_mode` (bool): Habilitar modo de recuperación (default: true)
- `debug_mode` (bool): Habilitar ventanas de debug (default: true)

## Topics ROS

### Suscripciones
- `/video_source/compressed` (sensor_msgs/CompressedImage): Imagen de entrada de la cámara

### Publicaciones
- `/cmd_vel` (geometry_msgs/Twist): Comandos de velocidad para el robot

## Algoritmo

### 1. Preprocesamiento de imagen
- Rotación de imagen 180°
- Extracción de ROI (Región de Interés) inferior-central
- Conversión a escala de grises
- Blur gaussiano
- Umbralización adaptativa y global combinadas
- Detección de bordes Canny (opcional)
- Operaciones morfológicas (erosión y dilatación)

### 2. Detección de línea
- Análisis de componentes conectados
- Filtrado por área
- Selección del mejor candidato basado en puntuación (área/distancia al centro)
- Ajuste de línea con `cv2.fitLine()`
- Cálculo de centroide y ángulo

### 3. Filtrado temporal
- Suavizado exponencial de centroide y ángulo
- Sistema de confianza basado en área detectada
- Manejo de pérdida temporal de línea

### 4. Control PID dual
- **Control de posición**: Error lateral respecto al centro
- **Control de ángulo**: Error angular respecto al objetivo
- Combinación ponderada de ambos controles
- Saturación de velocidades

### 5. Modo de recuperación
- Activación cuando se pierde la línea por tiempo prolongado
- Giro en dirección del último error conocido
- Velocidad reducida durante recuperación

## Debug y visualización

Cuando `debug_mode` está habilitado, se muestran ventanas con:
- **Imagen Original**: ROI extraído sin procesar
- **Procesamiento**: Imagen binaria después del procesamiento
- **Control**: Imagen con centroide detectado marcado
- **Bordes**: Resultado de detección de bordes (si está habilitada)

Las ventanas se escalan automáticamente para mejor visualización.

## Consejos de uso

### Ajuste inicial
1. Habilita `debug_mode` para ver el procesamiento
2. Ajusta `vision_height` y `roi_center_width` según tu configuración
3. Calibra `threshold_value` para tu iluminación
4. Ajusta `min_area` y `max_area` según el grosor de tu línea

### Optimización de control
1. Comienza con gains bajos y aumenta gradualmente
2. Usa `kp` para respuesta básica
3. Añade `kd` para reducir oscilaciones
4. Usa `ki` solo si hay error persistente
5. Ajusta `angle_weight` para balance posición/orientación

### Solución de problemas
- **Robot oscila mucho**: Reduce gains PID o aumenta `memory_factor`
- **Respuesta lenta**: Aumenta `kp` o reduce `memory_factor`
- **Pierde línea frecuentemente**: Ajusta parámetros de visión
- **No detecta línea**: Verifica `threshold_value` y iluminación

## Estructura del paquete

```
line_follower/
├── package.xml              # Metadatos del paquete
├── setup.py                 # Configuración de instalación
├── setup.cfg                # Configuración adicional
├── README.md                # Este archivo
├── resource/
│   └── line_follower        # Archivo marcador de ament
└── line_follower/
    ├── __init__.py          # Inicialización del paquete
    └── line_follower.py     # Nodo principal
```

## Autor

Jordan Palafox (a00835705@tec.mx)

## Licencia

Apache License 2.0 