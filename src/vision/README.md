# Paquete Vision - Puzzlebot (Versión Simplificada)

Este paquete detecta señales de tráfico usando YOLO e imprime qué estados enviaría a la máquina de estados, sin integración real.

## Características

- **Detección en tiempo real**: Utiliza YOLOv8 para detectar 7 tipos de señales de tráfico
- **Impresión de estados**: Muestra qué estado enviaría según las señales detectadas
- **Control de estabilidad**: Requiere múltiples detecciones consecutivas antes de confirmar
- **Modo debug visual**: Ventana con detecciones y información en tiempo real

## Clases de Señales Detectadas

| ID | Clase | Estado que Enviaría | Descripción |
|----|-------|------------------|-------------|
| 0 | `circle` | `idle` | Detener y esperar |
| 1 | `forward` | `followLineFast` | Seguir línea rápido |
| 2 | `give_way` | `followLineSlow` | Seguir línea despacio |
| 3 | `left` | `turnLeft` | Girar a la izquierda |
| 4 | `right` | `turnRight` | Girar a la derecha |
| 5 | `stop` | `stop` | Detenerse completamente |
| 6 | `working` | `followLineSlow` | Zona de trabajo - velocidad reducida |

## Arquitectura Simplificada

```
┌─────────────────────┐    ┌──────────────────────┐
│   Cámara            │    │  TrafficSignDetector │
│ (/video_source/     ├───►│                      │
│  compressed)        │    │  - Detección YOLO    │
└─────────────────────┘    │  - Filtrado          │
                           │  - IMPRESIÓN ESTADOS │
                           └──────────────────────┘
```

## Nodo Principal

### TrafficSignDetector

**Descripción**: Detecta señales de tráfico en tiempo real e imprime qué estados enviaría.

**Tópicos suscritos**:
- `/video_source/compressed` (sensor_msgs/CompressedImage): Stream de video de la cámara

**No publica nada** - Solo imprime en los logs.

**Parámetros**:
- `model_path` (string): Ruta al modelo YOLO entrenado (default: "traffic_signs_puzzlebot_v2.pt")
- `conf_threshold` (double): Umbral mínimo de confianza (default: 0.6)
- `show_debug_window` (bool): Mostrar ventana de debug con detecciones (default: true)

## Instalación

1. **Instalar dependencias**:
```bash
pip install ultralytics opencv-python
```

2. **Construir el paquete**:
```bash
cd ~/Puzzlebot-Autonomous-Scaled-Car
colcon build --packages-select vision
source install/setup.bash
```

## Uso

### Uso Básico

```bash
ros2 run vision traffic_sign_detector
```

### Uso con Parámetros

```bash
# Cambiar ruta del modelo y umbral de confianza
ros2 run vision traffic_sign_detector --ros-args \
  -p model_path:="/ruta/completa/a/tu/modelo.pt" \
  -p conf_threshold:=0.7 \
  -p show_debug_window:=false
```

### Ejemplo de Salida

```
[INFO] [traffic_sign_detector]: ✅ Traffic Sign Detector Node Inicializado
[INFO] [traffic_sign_detector]:    - Umbral de confianza: 0.6
[INFO] [traffic_sign_detector]:    - Ventana de debug: True
[INFO] [traffic_sign_detector]:    - Detecciones requeridas: 3
[INFO] [traffic_sign_detector]:    - Modo: Solo impresión de estados (sin envío)

[INFO] [traffic_sign_detector]: 📊 Detectando: STOP -> stop (Confianza: 0.85) [1/3]
[INFO] [traffic_sign_detector]: 📊 Detectando: STOP -> stop (Confianza: 0.87) [2/3]
[INFO] [traffic_sign_detector]: 🎯 ESTADO CONFIRMADO: STOP -> stop (Confianza: 0.89) [3/3]

[INFO] [traffic_sign_detector]: 📊 Detectando: LEFT -> turnLeft (Confianza: 0.76) [1/3]
[INFO] [traffic_sign_detector]: 🔄 Sin detecciones - contador reseteado
```

## Configuración

### Ajustar Detecciones Requeridas

En `traffic_sign_detector.py`, línea ~62:
```python
self.required_detections = 3  # Cambiar según necesidad
```

### Modificar Mapeo de Señales a Estados

En `traffic_sign_detector.py`, líneas ~50-57:
```python
self.signal_to_state = {
    'stop': 'stop',
    'left': 'turnLeft',
    'right': 'turnRight',
    'forward': 'followLineFast',
    'give_way': 'followLineSlow',
    'working': 'followLineSlow',
    'circle': 'idle'
}
```

## Debugging

### Logs Detallados

```bash
# Ejecutar con logs de debug para ver todas las detecciones
ros2 run vision traffic_sign_detector --ros-args --log-level debug
```

### Verificar Stream de Cámara

```bash
# Verificar que la cámara esté funcionando
ros2 topic echo /video_source/compressed
```

## Solución de Problemas

### Modelo No Encontrado
```
❌ Modelo no encontrado en: traffic_signs_puzzlebot_v2.pt
```
**Solución**: 
- Colocar el modelo en el directorio actual, o
- Usar ruta completa: `-p model_path:="/ruta/completa/al/modelo.pt"`

### Sin Detecciones
- Verificar que la cámara esté funcionando
- Ajustar el umbral de confianza con `conf_threshold`
- Verificar iluminación y calidad de las señales
- Revisar que el modelo sea compatible con tus señales

### Ventana de Debug No Aparece
```bash
# Asegúrate de que tienes entorno gráfico y ejecuta:
export DISPLAY=:0
ros2 run vision traffic_sign_detector
```

## Ejemplo de Integración Futura

Para integrar con la máquina de estados más adelante, simplemente descomenta las líneas de publicación en el código y añade:

```python
# Publicador de comandos de estado
self.state_pub = self.create_publisher(String, 'state_command', 10)

# En check_and_print_state(), después de confirmar:
msg = String()
msg.data = potential_state
self.state_pub.publish(msg)
```

## Licencia

MIT License 