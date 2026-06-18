import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CapyTownDriver(Node):
    def __init__(self):
        super().__init__('capytown_vision_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        
        # --- CONTROLADOR PD (Proporcional - Derivativo) ---
        self.base_speed = 0.15  
        self.kp = 0.008         # Fuerza de empuje hacia el centroide
        self.kd = 0.05          # Freno derivativo: evita giros bruscos y sobreimpulso (overshoot)
        
        self.last_error = 0.0   # Memoria para el cálculo del Kd y la recuperación

        # --- CAPTURA DE CÁMARA ---
        # Inicializa la cámara por hardware (Dispositivo 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("¡ERROR CRÍTICO! No se pudo acceder a la cámara en el index 0.")
        
        # --- TIMER DE ROS 2 (Sustituye al bucle 'while True') ---
        # Llama a la función de procesamiento automáticamente cada 0.033 segundos (~30 FPS)
        self.timer = self.create_timer(0.033, self.process_frame_callback)
        self.get_logger().info("Nodo de Prueba de Visión iniciado con Timer activo a ~30 FPS.")

    def drive(self, error_px):
        """Calcula y envía la velocidad usando PD para giros suaves"""
        # 1. Cálculo de la derivada (Cambio del error respecto al frame anterior)
        derivative = error_px - self.last_error
        self.last_error = error_px
        
        # 2. Fórmula PD: u(t) = Kp*e(t) + Kd*de/dt
        control_signal = (error_px * self.kp) + (derivative * self.kd)
        
        # 3. Límite de seguridad anti-sacudidas bruscas (max 1.5 rad/s)
        safe_angular_z = np.clip(control_signal, -1.5, 1.5)
        
        # Asignar velocidades al mensaje Twist
        self.twist.linear.x = self.base_speed
        self.twist.angular.z = safe_angular_z
        
        # Publicar en /cmd_vel
        self.cmd_pub.publish(self.twist)

    def process_frame_callback(self):
        """Captura un frame, procesa máscaras HSV y toma la acción de control"""
        ret, frame = self.cap.read()
        if not ret:
            return

        # Ajustar dimensiones de la imagen a 640x480
        frame = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 1. SEGMENTACIÓN POR COLOR (Filtros HSV)
        # Filtro Amarillo
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Filtro Blanco
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 2. ANÁLISIS DE LA REGIÓN DE INTERÉS (Línea de escaneo 360)
        roi_row = 360
        LANE_WIDTH_PX = 320 # Ancho nominal esperado entre líneas en píxeles

        yellow_line = mask_yellow[roi_row, :]
        white_line = mask_white[roi_row, :]

        # Detectar píxeles válidos en la fila de escaneo
        idx_yellow = np.where(yellow_line > 0)[0]
        idx_white = np.where(white_line > 0)[0]

        # Calcular el centroide de cada color detectado
        cx_y = int(np.mean(idx_yellow)) if len(idx_yellow) > 0 else None
        cx_w = int(np.mean(idx_white)) if len(idx_white) > 0 else None

        # Preparar lienzo para pintar el Dashboard gráfico
        res_display = frame.copy()
        cv2.line(res_display, (0, roi_row), (640, roi_row), (255, 0, 0), 2) # Línea azul ROI

        cx_lane = None

        # 3. FUSIÓN LOGICA DE RECONOCIMIENTO Y PROYECCIÓN
        if cx_y is not None and cx_w is not None:
            # Caso ideal: Ve ambos bordes del circuito
            cx_lane = int((cx_y + cx_w) / 2)
            cv2.putText(res_display, "SEGUIMIENTO (AMBOS BORDES)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.circle(res_display, (cx_y, roi_row), 5, (0, 255, 255), -1)
            cv2.circle(res_display, (cx_w, roi_row), 5, (255, 255, 255), -1)
            
        elif cx_y is not None:
            # Ve Amarillo (Derecha): El centro está hacia la izquierda (-)
            cx_lane = cx_y - int(LANE_WIDTH_PX / 2)
            cv2.putText(res_display, "PROYECTA (SOLO AMARILLO)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.circle(res_display, (cx_y, roi_row), 5, (0, 255, 255), -1)
            
        elif cx_w is not None:
            # Ve Blanco (Izquierda): El centro está hacia la derecha (+)
            cx_lane = cx_w + int(LANE_WIDTH_PX / 2)
            cv2.putText(res_display, "PROYECTA (SOLO BLANCO)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.circle(res_display, (cx_w, roi_row), 5, (255, 255, 255), -1)

        # 4. ACTUACIÓN DE MOVIMIENTO
        if cx_lane is not None:
            # Error respecto al centro de la cámara (320px)
            error_px = cx_lane - 320
            
            # Ejecutar control PD de velocidad
            self.drive(error_px)

            # Dibujar marcas del error calculado en pantalla
            cv2.line(res_display, (cx_lane, 480), (cx_lane, 0), (0, 255, 0), 2)
            cv2.circle(res_display, (cx_lane, roi_row), 8, (0, 0, 255), -1)
            cv2.putText(res_display, f"Error: {error_px} px", (cx_lane + 15, roi_row), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # Parar el robot si no encuentra ninguna línea por seguridad
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            cv2.putText(res_display, "FUERA DE CARRIL - DETENIDO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Renderizar interfaz en la pantalla de la Raspberry Pi / PC remoto
        cv2.imshow("Dashboard Prueba CapyTown", res_display)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    robot_node = CapyTownDriver()
    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        robot_node.get_logger().info("Nodo de prueba finalizado por usuario.")
    finally:
        # Apagado ordenado de periféricos
        robot_node.cap.release()
        cv2.destroyAllWindows()
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()