import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import matplotlib.pyplot as plt

class CapyTownDriver(Node):
    def __init__(self):
        super().__init__('capytown_vision_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        
        # --- CONTROLADOR PD ---
        self.base_speed = 0.12  
        self.kp = 0.008         
        self.kd = 0.05          
        self.last_error = 0.0   

        # --- TELEMETRÍA (Todo convertido a CM para las gráficas) --
        self.error_history_cm = [] 
        self.time_history = []
        self.start_time = time.time()
        self.last_time = time.time()
        
        # Factor físico: 21 cm reales equivalen a 320 px virtuales
        self.PX_TO_CM = 21.0 / 320.0
        
        # Posición geométrica interna (en metros para ROS)
        self.x_path_m = [0.0]
        self.y_path_m = [0.0]
        self.theta = 0.0

        # --- CAPTURA DE CÁMARA ONDEMAND ---
        # Abrimos la cámara local (Video Index 0 de la Raspberry Pi / PC)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("¡ERROR CRÍTICO! No se pudo abrir la cámara index 0.")
        
        # --- TIMER DE ROS 2 (REEMPLAZA AL 'WHILE TRUE') ---
        # Ejecuta la función 'process_frame_callback' cada 0.033 segundos (~30 FPS)
        self.timer = self.create_timer(0.033, self.process_frame_callback)
        self.get_logger().info("Nodo CapyTown Visión iniciado correctamente con Timer a 30 FPS.")

    def update_odometry(self):
        """Calcula matemáticamente la trayectoria X,Y en metros"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        v = self.twist.linear.x
        w = self.twist.angular.z

        # Integración numérica básica de odometría diferencial
        self.theta += w * dt
        self.x_path_m.append(self.x_path_m[-1] + v * np.cos(self.theta) * dt)
        self.y_path_m.append(self.y_path_m[-1] + v * np.sin(self.theta) * dt)

    def drive(self, error_px):
        """Calcula y envía la velocidad angular usando control PD"""
        derivative = error_px - self.last_error
        self.last_error = error_px

        control_signal = (error_px * self.kp) + (derivative * self.kd)
        safe_angular_z = np.clip(control_signal, -1.5, 1.5)

        self.twist.linear.x = self.base_speed
        self.twist.angular.z = safe_angular_z
        self.cmd_pub.publish(self.twist)

        # Actualizar telemetría interna
        self.update_odometry()
        current_duration = time.time() - self.start_time
        self.time_history.append(current_duration)
        self.error_history_cm.append(error_px * self.PX_TO_CM)

    def process_frame_callback(self):
        """Callback del Timer: Captura, procesa un frame y toma decisiones de control"""
        ret, frame = self.cap.read()
        if not ret:
            return

        # Redimensionar estándar a 640x480
        frame = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Máscaras de Color (Rangos HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Definición de la Región de Interés (ROI) - Fila 360
        roi_row = 360
        LANE_WIDTH_PX = 320 # Ancho estimado del carril en píxeles

        # Extraer líneas de escaneo en la ROI
        yellow_line = mask_yellow[roi_row, :]
        white_line = mask_white[roi_row, :]

        # Encontrar índices donde se detectan los colores
        idx_yellow = np.where(yellow_line > 0)[0]
        idx_white = np.where(white_line > 0)[0]

        cx_y = int(np.mean(idx_yellow)) if len(idx_yellow) > 0 else None
        cx_w = int(np.mean(idx_white)) if len(idx_white) > 0 else None

        # Copia para dibujar el Dashboard en vivo
        res_display = frame.copy()
        cv2.line(res_display, (0, roi_row), (640, roi_row), (255, 0, 0), 2) # Línea guía ROI

        cx_lane = None

        # Lógica de Fusión / Proyección geométrica
        if cx_y is not None and cx_w is not None:
            cx_lane = int((cx_y + cx_w) / 2)
            cv2.putText(res_display, "SEGUIMIENTO COMPLETO (AMBOS)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.circle(res_display, (cx_y, roi_row), 5, (0, 255, 255), -1)
            cv2.circle(res_display, (cx_w, roi_row), 5, (255, 255, 255), -1)
            
        elif cx_y is not None:
            cx_lane = cx_y - int(LANE_WIDTH_PX / 2)
            cv2.putText(res_display, "PROYECTANDO (SOLO AMARILLO)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.circle(res_display, (cx_y, roi_row), 5, (0, 255, 255), -1)
            
        elif cx_w is not None:
            cx_lane = cx_w + int(LANE_WIDTH_PX / 2)
            cv2.putText(res_display, "PROYECTANDO (SOLO BLANCO)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.circle(res_display, (cx_w, roi_row), 5, (255, 255, 255), -1)

        # Si detectamos el carril, actuamos con el controlador
        if cx_lane is not None:
            error_px = cx_lane - 320
            self.drive(error_px)

            # Dibujar elementos visuales del error en el dashboard
            error_cm_vivo = error_px * self.PX_TO_CM
            cv2.line(res_display, (cx_lane, 480), (cx_lane, 0), (0, 255, 0), 2)
            cv2.circle(res_display, (cx_lane, roi_row), 8, (0, 0, 255), -1)
            cv2.putText(res_display, f"Error: {error_cm_vivo:.1f} cm", (cx_lane + 15, roi_row), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # Si se pierde la línea, se detiene por seguridad
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            cv2.putText(res_display, "LINEA PERDIDA - PARADA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Mostrar interfaz gráfica de OpenCV
        cv2.imshow("Dashboard CapyTown", res_display)
        cv2.waitKey(1) # Refresca las ventanas internas de la UI de OpenCV

def main(args=None):
    rclpy.init(args=args)
    robot_node = CapyTownDriver()
    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        robot_node.get_logger().info("Apagando nodo por interrupción de teclado...")
    finally:
        # Cerrar recursos de gráficos y cámara de manera segura
        robot_node.cap.release()
        cv2.destroyAllWindows()
        
        # Graficar telemetría final al detener el script
        if len(robot_node.time_history) > 0:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
            
            ax1.plot(robot_node.time_history, robot_node.error_history_cm, 'r-', label='Error (cm)')
            ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            ax1.set_title("Evolución del Error de Guiado")
            ax1.set_xlabel("Tiempo (s)")
            ax1.set_ylabel("Error (cm)")
            ax1.grid(True)
            ax1.legend()

            ax2.plot(robot_node.x_path_m, robot_node.y_path_m, 'b-', label='Trayectoria Odometria')
            ax2.set_title("Estimación de Trayectoria Recorregida")
            ax2.set_xlabel("X (metros)")
            ax2.set_ylabel("Y (metros)")
            ax2.axis('equal')
            ax2.grid(True)
            ax2.legend()

            plt.tight_layout()
            print("\n[Telemetría] Mostrando gráficas del experimento. Cierra la ventana de gráficos para salir completamente.")
            plt.show()

        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()