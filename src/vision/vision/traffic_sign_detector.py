#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import os
import time

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        
        # Parámetros configurables
        self.declare_parameter('model_path', '/home/jordan/Puzzlebot-Autonomous-Scaled-Car/src/vision/models/traffic_signs_puzzlebot_v3.pt')
        self.declare_parameter('conf_threshold', 0.6)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('default_state', 'followLineFast')
        self.declare_parameter('no_detection_timeout', 3.0)
        self.declare_parameter('giveway_stop_duration', 5.0)
        self.declare_parameter('forward_move_duration', 6.0)  # Duración para movimiento recto
        self.declare_parameter('enable_color_detection', True)
        self.declare_parameter('color_min_area', 1000)
        
        # Obtener parámetros
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.show_debug = self.get_parameter('show_debug_window').get_parameter_value().bool_value
        self.default_state = self.get_parameter('default_state').get_parameter_value().string_value
        self.no_detection_timeout = self.get_parameter('no_detection_timeout').get_parameter_value().double_value
        self.giveway_stop_duration = self.get_parameter('giveway_stop_duration').get_parameter_value().double_value
        self.forward_move_duration = self.get_parameter('forward_move_duration').get_parameter_value().double_value
        self.enable_color_detection = self.get_parameter('enable_color_detection').get_parameter_value().bool_value
        self.color_min_area = self.get_parameter('color_min_area').get_parameter_value().integer_value
        
        # Verificar que el modelo existe
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Modelo no encontrado en: {model_path}")
            raise FileNotFoundError(f"Modelo no encontrado: {model_path}")
        
        # Cargar modelo YOLO
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ Modelo YOLO cargado: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando modelo: {e}")
            raise
        
        self.bridge = CvBridge()
        
        # Definir las 7 clases de señales (0–6)
        self.classes = {
            0: 'circle',
            1: 'forward', 
            2: 'give_way',
            3: 'left',
            4: 'right',
            5: 'stop',
            6: 'working'
        }
        
        # Mapeo de señales a estados
        self.signal_to_state = {
            'stop': 'stop',
            'left': 'turnLeft',
            'right': 'turnRight',
            'forward': 'moveStraight',  # Forward ahora usa moveStraight
            'give_way': 'stop',  # Give way primero se detiene
            'working': 'followLineSlow',
            'circle': 'idle'
        }
        
        # Mapeo de colores a estados
        self.color_to_state = {
            'red': 'stop',
            'yellow': 'followLineSlow',
            'green': 'followLineFast'
        }
        
        # Rangos HSV para detección de colores
        self.color_ranges = {
            'red': [
                (np.array([0, 50, 50]), np.array([10, 255, 255])),      # Rojo bajo
                (np.array([170, 50, 50]), np.array([180, 255, 255]))   # Rojo alto
            ],
            'yellow': [
    (np.array([20, 150, 150]), np.array([30, 255, 255]))
                ],
            'green': [
                (np.array([40, 50, 50]), np.array([80, 255, 255]))     # Verde
            ]
        }
        
        # Estado de detección para estabilidad
        self.last_detection = None
        self.detection_count = 0
        self.required_detections = 50  # Para señales de tráfico
        self.required_detections_traffic_light = 4  # Para semáforo (círculo + color)
        self.last_detection_time = time.time()
        self.current_published_state = None
        self.detection_type = None  # 'signal', 'color', o 'traffic_light'
        
        # Variables específicas para give_way
        self.giveway_active = False
        self.giveway_start_time = None
        self.giveway_timer = None
        
        # Variables específicas para forward
        self.forward_active = False
        self.forward_start_time = None
        self.forward_timer = None
        
        # Suscriptor a imágenes comprimidas
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # Publicador de comandos de estado
        self.state_pub = self.create_publisher(String, 'state_command', 10)
        
        # Timer para verificar timeout y enviar estado por defecto
        self.default_timer = self.create_timer(0.5, self.check_default_state)
        
        self.get_logger().info("✅ Traffic Sign Detector Node Inicializado")
        self.get_logger().info(f"   - Umbral de confianza: {self.conf_threshold}")
        self.get_logger().info(f"   - Ventana de debug: {self.show_debug}")
        self.get_logger().info(f"   - Detecciones requeridas señales: {self.required_detections}")
        self.get_logger().info(f"   - Detecciones requeridas semáforo: {self.required_detections_traffic_light}")
        self.get_logger().info(f"   - Estado por defecto: {self.default_state}")
        self.get_logger().info(f"   - Timeout sin detección: {self.no_detection_timeout}s")
        self.get_logger().info(f"   - Duración stop give_way: {self.giveway_stop_duration}s")
        self.get_logger().info(f"   - Duración move forward: {self.forward_move_duration}s")
        self.get_logger().info(f"   - Detección de colores: {self.enable_color_detection}")
        self.get_logger().info(f"   - Área mínima colores: {self.color_min_area}px²")
        self.get_logger().info("   - Modo: Publicando comandos de estado")
        
        # Enviar estado inicial
        self.publish_state(self.default_state, "Estado inicial")

    def detect_colors(self, frame):
        """Detectar colores en el frame"""
        if not self.enable_color_detection:
            return []
        
        color_detections = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        for color_name, ranges in self.color_ranges.items():
            # Crear máscara combinando todos los rangos para este color
            mask = None
            for lower, upper in ranges:
                current_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask)
            
            # Aplicar operaciones morfológicas para limpiar la máscara
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Encontrar contornos
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.color_min_area:
                    # Calcular centro y bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    color_detections.append({
                        'color': color_name,
                        'area': area,
                        'center': (center_x, center_y),
                        'bbox': (x, y, x + w, y + h),
                        'confidence': min(1.0, area / (self.color_min_area * 3))  # Confianza basada en área
                    })
                    
                    self.get_logger().debug(
                        f"🎨 Color detectado: {color_name.upper()} "
                        f"| Área: {area}px² "
                        f"| Centro: ({center_x}, {center_y})"
                    )
        
        return color_detections

    def detect_traffic_lights(self, signal_detections, color_detections):
        """Detectar semáforos: combinar círculos detectados por YOLO con colores específicos"""
        traffic_light_detections = []
        
        # Buscar círculos en las detecciones de señales
        circles = [det for det in signal_detections if det['class'] == 'circle']
        
        if not circles or not color_detections:
            return traffic_light_detections
        
        # Para cada círculo, buscar colores de semáforo cercanos
        for circle in circles:
            circle_x1, circle_y1, circle_x2, circle_y2 = circle['bbox']
            circle_center_x = (circle_x1 + circle_x2) // 2
            circle_center_y = (circle_y1 + circle_y2) // 2
            circle_area = (circle_x2 - circle_x1) * (circle_y2 - circle_y1)
            
            # Buscar colores de semáforo dentro o cerca del círculo
            for color_det in color_detections:
                if color_det['color'] in ['red', 'yellow', 'green']:
                    color_center_x, color_center_y = color_det['center']
                    
                    # Calcular distancia entre círculo y color
                    distance = ((circle_center_x - color_center_x) ** 2 + 
                               (circle_center_y - color_center_y) ** 2) ** 0.5
                    
                    # Radio del círculo (aproximado)
                    circle_radius = min(circle_x2 - circle_x1, circle_y2 - circle_y1) // 2
                    
                    # Si el color está dentro o muy cerca del círculo (tolerancia 50% extra)
                    if distance <= circle_radius * 1.5:
                        traffic_light_detections.append({
                            'type': 'traffic_light',
                            'color': color_det['color'],
                            'confidence': min(circle['confidence'], color_det['confidence']),
                            'circle_bbox': circle['bbox'],
                            'color_bbox': color_det['bbox'],
                            'distance': distance
                        })
                        
                        self.get_logger().debug(
                            f"🚦 Semáforo detectado: Círculo + {color_det['color'].upper()} "
                            f"| Distancia: {distance:.1f}px "
                            f"| Confianza: {min(circle['confidence'], color_det['confidence']):.2f}"
                        )
                        break  # Solo un color por círculo
        
        return traffic_light_detections

    def publish_state(self, state, reason):
        """Publicar comando de estado"""
        if self.current_published_state != state:
            msg = String()
            msg.data = state
            self.state_pub.publish(msg)
            self.current_published_state = state
            self.get_logger().info(f"📤 Estado enviado: {state} ({reason})")

    def start_giveway_sequence(self):
        """Iniciar secuencia especial de give_way"""
        if not self.giveway_active:
            self.giveway_active = True
            self.giveway_start_time = time.time()
            
            # Cancelar timer anterior si existe
            if self.giveway_timer is not None:
                self.giveway_timer.cancel()
            
            # Crear timer para regresar a followLineFast después de 5 segundos
            self.giveway_timer = self.create_timer(
                self.giveway_stop_duration, 
                self.end_giveway_sequence
            )
            
            # Enviar comando de stop
            self.publish_state('stop', f"Give way - deteniéndose por {self.giveway_stop_duration}s")
            
            self.get_logger().info(f"🚦 Iniciando secuencia GIVE_WAY: stop por {self.giveway_stop_duration}s")

    def end_giveway_sequence(self):
        """Terminar secuencia de give_way y regresar a followLineFast"""
        if self.giveway_active:
            self.giveway_active = False
            self.giveway_start_time = None
            
            # Cancelar timer
            if self.giveway_timer is not None:
                self.giveway_timer.cancel()
                self.giveway_timer = None
            
            # Regresar a followLineFast
            self.publish_state('followLineFast', "Give way completado - regresando a seguir línea")
            
            self.get_logger().info("✅ Secuencia GIVE_WAY completada - regresando a followLineFast")

    def start_forward_sequence(self):
        """Iniciar secuencia especial de forward"""
        if not self.forward_active:
            self.forward_active = True
            self.forward_start_time = time.time()
            
            # Cancelar timer anterior si existe
            if self.forward_timer is not None:
                self.forward_timer.cancel()
            
            # Crear timer para regresar a followLineFast después de la duración especificada
            self.forward_timer = self.create_timer(
                self.forward_move_duration, 
                self.end_forward_sequence
            )
            
            # Enviar comando de moveStraight
            self.publish_state('moveStraight', f"Forward - avanzando recto por {self.forward_move_duration}s")
            
            self.get_logger().info(f"➡️ Iniciando secuencia FORWARD: moveStraight por {self.forward_move_duration}s")

    def end_forward_sequence(self):
        """Terminar secuencia de forward y regresar a followLineFast"""
        if self.forward_active:
            self.forward_active = False
            self.forward_start_time = None
            
            # Cancelar timer
            if self.forward_timer is not None:
                self.forward_timer.cancel()
                self.forward_timer = None
            
            # Regresar a followLineFast
            self.publish_state('followLineFast', "Forward completado - regresando a seguir línea")
            
            self.get_logger().info("✅ Secuencia FORWARD completada - regresando a followLineFast")

    def check_default_state(self):
        """Verificar si debe enviar estado por defecto debido a timeout"""
        # No interferir si estamos en secuencias especiales
        if self.giveway_active or self.forward_active:
            return
            
        time_since_detection = time.time() - self.last_detection_time
        
        if time_since_detection > self.no_detection_timeout:
            if self.current_published_state != self.default_state:
                self.publish_state(self.default_state, f"Sin detecciones por {time_since_detection:.1f}s")

    def process_frame(self, frame):
        """Procesar frame con YOLO y detección de colores"""
        signal_detections = []
        
        # Ejecutar inferencia YOLO
        results = self.model(frame)
        
        # Procesar resultados de YOLO
        for result in results:
            for box in result.boxes:
                conf = float(box.conf)
                if conf >= self.conf_threshold:
                    cls_id = int(box.cls)
                    cls_name = self.classes.get(cls_id, "unknown")
                    
                    # Obtener coordenadas del bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    detection = {
                        'type': 'signal',
                        'class': cls_name,
                        'confidence': conf,
                        'bbox': (x1, y1, x2, y2)
                    }
                    signal_detections.append(detection)
                    
                    # Log de detección
                    self.get_logger().debug(
                        f"🚦 Señal detectada: {cls_name.upper()} "
                        f"| Confianza: {conf:.2f} "
                        f"| Posición: ({x1}, {y1}) a ({x2}, {y2})"
                    )
                    
                    # Dibujar en el frame si el debug está activado
                    if self.show_debug:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            f"{cls_name} {conf:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )
        
        # Detectar colores
        color_detections = self.detect_colors(frame)
        
        # Detectar semáforos (círculos + colores específicos)
        traffic_light_detections = self.detect_traffic_lights(signal_detections, color_detections)
        
        # Dibujar detecciones de semáforos si el debug está activado
        if self.show_debug:
            for tl_det in traffic_light_detections:
                x1, y1, x2, y2 = tl_det['circle_bbox']
                color_name = tl_det['color']
                confidence = tl_det['confidence']
                
                # Color para dibujar según el semáforo
                tl_color_bgr = {
                    'red': (0, 0, 255),
                    'yellow': (0, 255, 255), 
                    'green': (0, 255, 0)
                }
                
                # Dibujar círculo del semáforo con borde grueso
                cv2.rectangle(frame, (x1, y1), (x2, y2), tl_color_bgr.get(color_name, (255, 255, 255)), 4)
                cv2.putText(
                    frame,
                    f"SEMAFORO {color_name.upper()} {confidence:.2f}",
                    (x1, y1 - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    tl_color_bgr.get(color_name, (255, 255, 255)),
                    2
                )
        
        # Dibujar detecciones de colores si el debug está activado
        if self.show_debug:
            for color_det in color_detections:
                x1, y1, x2, y2 = color_det['bbox']
                color_name = color_det['color']
                area = color_det['area']
                
                # Colores para dibujar
                color_bgr = {
                    'red': (0, 0, 255),
                    'yellow': (0, 255, 255),
                    'green': (0, 255, 0)
                }
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color_bgr.get(color_name, (255, 255, 255)), 2)
                cv2.putText(
                    frame,
                    f"{color_name} {area}px²",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color_bgr.get(color_name, (255, 255, 255)),
                    2
                )
        
        return signal_detections, color_detections, traffic_light_detections, frame

    def check_and_publish_state(self, signal_detections, color_detections, traffic_light_detections):
        """Verificar detecciones y publicar estado correspondiente"""
        # Si estamos en secuencias especiales, no procesar otras detecciones
        if self.giveway_active or self.forward_active:
            return
        
        # Nueva lógica de prioridad basada en semáforo (círculo + color):
        # 1. Si hay semáforo rojo o amarillo -> priorizar semáforo
        # 2. Si hay semáforo verde -> priorizar señales de tráfico
        # 3. Si no hay semáforo pero hay señales -> procesar señales
        # 4. Si solo hay colores sin círculo -> procesar colores normales
        
        # Analizar qué semáforos se detectaron (círculo + color)
        traffic_light_colors = [det['color'] for det in traffic_light_detections] if traffic_light_detections else []
        has_tl_red_or_yellow = any(color in ['red', 'yellow'] for color in traffic_light_colors)
        has_tl_green = 'green' in traffic_light_colors
        
        # Filtrar señales excluyendo círculos (ya procesados como semáforos)
        non_circle_signals = [det for det in signal_detections if det['class'] != 'circle']
        
        if has_tl_red_or_yellow:
            # PRIORIDAD 1: Semáforo rojo o amarillo - siempre tiene prioridad máxima
            self.get_logger().debug("🚦 Prioridad: SEMÁFORO ROJO/AMARILLO detectado - priorizando semáforo")
            self.process_traffic_light_detections(traffic_light_detections)
        elif has_tl_green and non_circle_signals:
            # PRIORIDAD 2: Semáforo verde + señales - priorizar señales de tráfico
            self.get_logger().debug("🚦 Prioridad: SEMÁFORO VERDE + señales - priorizando señales de tráfico")
            self.process_signal_detections(non_circle_signals)
        elif traffic_light_detections:
            # PRIORIDAD 3: Solo semáforo (cualquier color) - procesarlo
            self.get_logger().debug("🚦 Prioridad: Solo SEMÁFORO detectado")
            self.process_traffic_light_detections(traffic_light_detections)
        elif non_circle_signals:
            # PRIORIDAD 4: Solo señales de tráfico (sin círculos) - procesarlas
            self.get_logger().debug("🚦 Prioridad: Solo señales de tráfico detectadas")
            self.process_signal_detections(non_circle_signals)
        elif color_detections:
            # PRIORIDAD 5: Solo colores sin círculo - procesarlos como colores normales
            self.get_logger().debug("🚦 Prioridad: Solo colores detectados (sin círculo)")
            self.process_color_detections(color_detections)
        else:
            # No hay detecciones, resetear contador
            if self.detection_count > 0:
                self.get_logger().info("🔄 Sin detecciones - contador reseteado")
                self.detection_count = 0
                self.last_detection = None
                self.detection_type = None

    def process_traffic_light_detections(self, traffic_light_detections):
        """Procesar detecciones de semáforos (círculo + color)"""
        # Hay detecciones - actualizar tiempo
        self.last_detection_time = time.time()
        
        # Tomar la detección con mayor confianza
        best_detection = max(traffic_light_detections, key=lambda x: x['confidence'])
        detected_color = best_detection['color']
        confidence = best_detection['confidence']
        
        # Verificar si es la misma detección consecutiva del mismo tipo
        if detected_color == self.last_detection and self.detection_type == 'traffic_light':
            self.detection_count += 1
        else:
            self.detection_count = 1
            self.last_detection = detected_color
            self.detection_type = 'traffic_light'
        
        # Usar el umbral específico para semáforos (10 detecciones)
        required_count = self.required_detections_traffic_light
        
        # Procesar según el número de detecciones
        if detected_color in self.color_to_state:
            potential_state = self.color_to_state[detected_color]
            
            if self.detection_count >= required_count:
                # Suficientes detecciones - confirmar y enviar estado
                self.publish_state(potential_state, f"Semáforo confirmado: {detected_color.upper()}")
                
                self.get_logger().info(
                    f"🚦 SEMÁFORO CONFIRMADO: {detected_color.upper()} "
                    f"-> {potential_state} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{required_count}]"
                )
                # Resetear después de confirmar
                self.detection_count = 0
                self.last_detection = None
                self.detection_type = None
            else:
                # Detección en progreso
                self.get_logger().info(
                    f"📊 Detectando semáforo: {detected_color.upper()} "
                    f"-> {potential_state} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{required_count}]"
                )

    def process_signal_detections(self, signal_detections):
        """Procesar detecciones de señales YOLO"""
        # Hay detecciones - actualizar tiempo
        self.last_detection_time = time.time()
        
        # Tomar la detección con mayor confianza
        best_detection = max(signal_detections, key=lambda x: x['confidence'])
        detected_signal = best_detection['class']
        confidence = best_detection['confidence']
        
        # Verificar si es la misma detección consecutiva del mismo tipo
        if detected_signal == self.last_detection and self.detection_type == 'signal':
            self.detection_count += 1
        else:
            self.detection_count = 1
            self.last_detection = detected_signal
            self.detection_type = 'signal'
        
        # Procesar según el número de detecciones
        if detected_signal in self.signal_to_state:
            potential_state = self.signal_to_state[detected_signal]
            
            if self.detection_count >= self.required_detections:
                # Suficientes detecciones - confirmar y procesar
                if detected_signal == 'give_way':
                    # Caso especial para give_way
                    self.start_giveway_sequence()
                elif detected_signal == 'forward':
                    # Caso especial para forward
                    self.start_forward_sequence()
                else:
                    # Casos normales
                    self.publish_state(potential_state, f"Señal confirmada: {detected_signal.upper()}")
                
                # Mostrar resultado con formato especial para secuencias
                if detected_signal == 'give_way':
                    result_text = 'stop->followLineFast'
                elif detected_signal == 'forward':
                    result_text = 'moveStraight->followLineFast'
                else:
                    result_text = potential_state
                
                self.get_logger().info(
                    f"🎯 SEÑAL CONFIRMADA: {detected_signal.upper()} "
                    f"-> {result_text} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{self.required_detections}]"
                )
                # Resetear después de confirmar
                self.detection_count = 0
                self.last_detection = None
                self.detection_type = None
            else:
                # Detección en progreso
                if detected_signal == 'give_way':
                    result_text = 'stop->followLineFast'
                elif detected_signal == 'forward':
                    result_text = 'moveStraight->followLineFast'
                else:
                    result_text = potential_state
                
                self.get_logger().info(
                    f"📊 Detectando señal: {detected_signal.upper()} "
                    f"-> {result_text} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{self.required_detections}]"
                )

    def process_color_detections(self, color_detections):
        """Procesar detecciones de colores"""
        # Hay detecciones - actualizar tiempo
        self.last_detection_time = time.time()
        
        # Tomar la detección con mayor área (más confiable)
        best_detection = max(color_detections, key=lambda x: x['area'])
        detected_color = best_detection['color']
        confidence = best_detection['confidence']
        
        # Verificar si es la misma detección consecutiva del mismo tipo
        if detected_color == self.last_detection and self.detection_type == 'color':
            self.detection_count += 1
        else:
            self.detection_count = 1
            self.last_detection = detected_color
            self.detection_type = 'color'
        
        # Procesar según el número de detecciones
        if detected_color in self.color_to_state:
            potential_state = self.color_to_state[detected_color]
            
            if self.detection_count >= self.required_detections:
                # Suficientes detecciones - confirmar y enviar estado
                self.publish_state(potential_state, f"Color confirmado: {detected_color.upper()}")
                
                self.get_logger().info(
                    f"🎯 COLOR CONFIRMADO: {detected_color.upper()} "
                    f"-> {potential_state} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{self.required_detections}]"
                )
                # Resetear después de confirmar
                self.detection_count = 0
                self.last_detection = None
                self.detection_type = None
            else:
                # Detección en progreso
                self.get_logger().info(
                    f"📊 Detectando color: {detected_color.upper()} "
                    f"-> {potential_state} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{self.required_detections}]"
                )

    def image_callback(self, msg):
        """Callback del suscriptor de imágenes"""
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Procesar frame
            signal_detections, color_detections, traffic_light_detections, processed_frame = self.process_frame(cv_image)
            
            # Verificar y publicar estado
            self.check_and_publish_state(signal_detections, color_detections, traffic_light_detections)
            
            # Mostrar ventana de debug si está habilitada
            if self.show_debug:
                # Añadir información de estado en la imagen
                info_text = f"Ultima deteccion: {self.last_detection or 'Ninguna'} ({self.detection_type or 'N/A'})"
                cv2.putText(
                    processed_frame,
                    info_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )
                
                # Mostrar contador apropiado según el tipo de detección
                if self.detection_type == 'traffic_light':
                    count_text = f"Contador semáforo: {self.detection_count}/{self.required_detections_traffic_light}"
                else:
                    count_text = f"Contador: {self.detection_count}/{self.required_detections}"
                cv2.putText(
                    processed_frame,
                    count_text,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )
                
                # Estado actual publicado
                state_text = f"Estado actual: {self.current_published_state or 'Ninguno'}"
                cv2.putText(
                    processed_frame,
                    state_text,
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2
                )
                
                # Información especial de secuencias
                if self.giveway_active:
                    elapsed = time.time() - self.giveway_start_time
                    remaining = max(0, self.giveway_stop_duration - elapsed)
                    giveway_text = f"GIVE_WAY: {remaining:.1f}s restantes"
                    cv2.putText(
                        processed_frame,
                        giveway_text,
                        (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 165, 255),  # Color naranja
                        2
                    )
                elif self.forward_active:
                    elapsed = time.time() - self.forward_start_time
                    remaining = max(0, self.forward_move_duration - elapsed)
                    forward_text = f"FORWARD: {remaining:.1f}s restantes"
                    cv2.putText(
                        processed_frame,
                        forward_text,
                        (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 165, 0),  # Color azul
                        2
                    )
                else:
                    # Tiempo desde última detección
                    time_since = time.time() - self.last_detection_time
                    timeout_text = f"Sin deteccion: {time_since:.1f}s / {self.no_detection_timeout}s"
                    cv2.putText(
                        processed_frame,
                        timeout_text,
                        (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2
                    )
                
                # Información de detecciones activas y lógica de prioridad
                detection_info = f"Señales: {len(signal_detections)} | Colores: {len(color_detections)} | Semáforos: {len(traffic_light_detections)}"
                cv2.putText(
                    processed_frame,
                    detection_info,
                    (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (200, 200, 200),
                    2
                )
                
                # Mostrar lógica de prioridad activa
                if signal_detections or color_detections or traffic_light_detections:
                    traffic_light_colors = [det['color'] for det in traffic_light_detections] if traffic_light_detections else []
                    has_tl_red_or_yellow = any(color in ['red', 'yellow'] for color in traffic_light_colors)
                    has_tl_green = 'green' in traffic_light_colors
                    non_circle_signals = [det for det in signal_detections if det['class'] != 'circle']
                    
                    if has_tl_red_or_yellow:
                        priority_text = "Prioridad: SEMAFORO ROJO/AMARILLO"
                        priority_color = (0, 0, 255)  # Rojo
                    elif has_tl_green and non_circle_signals:
                        priority_text = "Prioridad: SEMAFORO VERDE->SEÑALES"
                        priority_color = (0, 255, 0)  # Verde
                    elif traffic_light_detections:
                        priority_text = "Prioridad: SEMAFORO"
                        priority_color = (0, 255, 255)  # Amarillo
                    elif non_circle_signals:
                        priority_text = "Prioridad: SEÑALES"
                        priority_color = (255, 255, 0)  # Cyan
                    elif color_detections:
                        priority_text = "Prioridad: COLORES (sin círculo)"
                        priority_color = (255, 0, 255)  # Magenta
                    
                    cv2.putText(
                        processed_frame,
                        priority_text,
                        (10, 180),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        priority_color,
                        2
                    )
                
                cv2.imshow("Traffic Sign Detection", processed_frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Error procesando imagen: {str(e)}")

    def destroy_node(self):
        """Limpieza al destruir el nodo"""
        # Cancelar timers si existen
        if self.giveway_timer is not None:
            self.giveway_timer.cancel()
        if self.forward_timer is not None:
            self.forward_timer.cancel()
            
        if self.show_debug:
            cv2.destroyAllWindows()
        self.get_logger().info("🔴 Traffic Sign Detector terminado")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrafficSignDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"❌ Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main() 