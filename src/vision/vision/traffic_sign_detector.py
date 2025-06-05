#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
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
        self.declare_parameter('model_path', '/home/jordan/Puzzlebot-Autonomous-Scaled-Car/src/vision/models/traffic_signs_puzzlebot_v2.pt')
        self.declare_parameter('conf_threshold', 0.6)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('default_state', 'followLineFast')
        self.declare_parameter('no_detection_timeout', 3.0)  # Segundos sin detección antes de estado por defecto
        
        # Obtener parámetros
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.show_debug = self.get_parameter('show_debug_window').get_parameter_value().bool_value
        self.default_state = self.get_parameter('default_state').get_parameter_value().string_value
        self.no_detection_timeout = self.get_parameter('no_detection_timeout').get_parameter_value().double_value
        
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
        
        # Definir las 7 clases (0–6)
        self.classes = {
            0: 'circle',
            1: 'forward', 
            2: 'give_way',
            3: 'left',
            4: 'right',
            5: 'stop',
            6: 'working'
        }
        
        # Mapeo de señales a estados que enviaría
        self.signal_to_state = {
            'stop': 'stop',
            'left': 'turnLeft',
            'right': 'turnRight',
            'forward': 'followLineFast',
            'give_way': 'followLineSlow',
            'working': 'followLineSlow',
            'circle': 'idle'
        }
        
        # Estado de detección para estabilidad
        self.last_detection = None
        self.detection_count = 0
        self.required_detections = 8  # Número de detecciones consecutivas requeridas
        self.last_detection_time = time.time()
        self.current_published_state = None
        
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
        self.get_logger().info(f"   - Detecciones requeridas: {self.required_detections}")
        self.get_logger().info(f"   - Estado por defecto: {self.default_state}")
        self.get_logger().info(f"   - Timeout sin detección: {self.no_detection_timeout}s")
        self.get_logger().info("   - Modo: Publicando comandos de estado")
        
        # Enviar estado inicial
        self.publish_state(self.default_state, "Estado inicial")

    def publish_state(self, state, reason):
        """Publicar comando de estado"""
        if self.current_published_state != state:
            msg = String()
            msg.data = state
            self.state_pub.publish(msg)
            self.current_published_state = state
            self.get_logger().info(f"📤 Estado enviado: {state} ({reason})")

    def check_default_state(self):
        """Verificar si debe enviar estado por defecto debido a timeout"""
        time_since_detection = time.time() - self.last_detection_time
        
        if time_since_detection > self.no_detection_timeout:
            if self.current_published_state != self.default_state:
                self.publish_state(self.default_state, f"Sin detecciones por {time_since_detection:.1f}s")

    def process_frame(self, frame):
        """Procesar frame con YOLO y retornar detecciones"""
        detections = []
        
        # Ejecutar inferencia
        results = self.model(frame)
        
        # Procesar resultados
        for result in results:
            for box in result.boxes:
                conf = float(box.conf)
                if conf >= self.conf_threshold:
                    cls_id = int(box.cls)
                    cls_name = self.classes.get(cls_id, "unknown")
                    
                    # Obtener coordenadas del bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    detection = {
                        'class': cls_name,
                        'confidence': conf,
                        'bbox': (x1, y1, x2, y2)
                    }
                    detections.append(detection)
                    
                    # Log de detección
                    self.get_logger().debug(
                        f"🔍 Detectado: {cls_name.upper()} "
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
        
        return detections, frame

    def check_and_publish_state(self, detections):
        """Verificar detecciones y publicar estado correspondiente"""
        if not detections:
            # No hay detecciones, resetear contador pero NO actualizar tiempo
            if self.detection_count > 0:
                self.get_logger().info("🔄 Sin detecciones - contador reseteado")
                self.detection_count = 0
                self.last_detection = None
            return
        
        # Hay detecciones - actualizar tiempo
        self.last_detection_time = time.time()
        
        # Tomar la detección con mayor confianza
        best_detection = max(detections, key=lambda x: x['confidence'])
        detected_signal = best_detection['class']
        confidence = best_detection['confidence']
        
        # Verificar si es la misma detección consecutiva
        if detected_signal == self.last_detection:
            self.detection_count += 1
        else:
            self.detection_count = 1
            self.last_detection = detected_signal
        
        # Procesar según el número de detecciones
        if detected_signal in self.signal_to_state:
            potential_state = self.signal_to_state[detected_signal]
            
            if self.detection_count >= self.required_detections:
                # Suficientes detecciones - confirmar y enviar estado
                self.publish_state(potential_state, f"Señal confirmada: {detected_signal.upper()}")
                self.get_logger().info(
                    f"🎯 ESTADO CONFIRMADO: {detected_signal.upper()} "
                    f"-> {potential_state} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{self.required_detections}]"
                )
                # Resetear después de confirmar
                self.detection_count = 0
                self.last_detection = None
            else:
                # Detección en progreso
                self.get_logger().info(
                    f"📊 Detectando: {detected_signal.upper()} "
                    f"-> {potential_state} "
                    f"(Confianza: {confidence:.2f}) "
                    f"[{self.detection_count}/{self.required_detections}]"
                )
        else:
            self.get_logger().warn(f"⚠️ Señal desconocida: {detected_signal}")

    def image_callback(self, msg):
        """Callback del suscriptor de imágenes"""
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Procesar frame
            detections, processed_frame = self.process_frame(cv_image)
            
            # Verificar y publicar estado
            self.check_and_publish_state(detections)
            
            # Mostrar ventana de debug si está habilitada
            if self.show_debug:
                # Añadir información de estado en la imagen
                info_text = f"Ultima deteccion: {self.last_detection or 'Ninguna'}"
                cv2.putText(
                    processed_frame,
                    info_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )
                
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
                
                cv2.imshow("Traffic Sign Detection", processed_frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Error procesando imagen: {str(e)}")

    def destroy_node(self):
        """Limpieza al destruir el nodo"""
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