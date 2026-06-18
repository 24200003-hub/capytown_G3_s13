# capytown_esan — Reto de Visión Artificial (Seguidor de Carril)

Paquete de ROS 2 (Humble, `ament_python`) desarrollado para el robot **Yahboom MicroROS-Pi5**. Este paquete implementa un algoritmo de visión artificial basado en segmentación de colores HSV y un controlador Proporcional-Derivativo (PD) para asegurar que el robot identifique y siga un circuito delimitado por líneas amarillas y blancas sin salirse del carril.

---

## 1. Estructura del Repositorio

El paquete está organizado bajo los estándares nativos de ROS 2:

```text
capytown_esan/
├── Presentacion/
│   └── Link PPT.txt              # Archivo de inicialización (Vacío)
├── capytown_esan/                # Código fuente del paquete (Módulo Python)
│   ├── __init__.py               # Archivo de inicialización (Vacío)
│   ├── square.py                 # Movimiento en cuadrado (Odom antiguo)
│   ├── calibrate_beff.py         # Calibración antigua del track efectivo
│   ├── color_imagen1.py          # CONTROLADOR PRINCIPAL + Telemetría y Gráficas
│    prueba.py                 # CONTROLADOR DE PRUEBA (Versión ligera)
├── config/
│   └── wheel_params.yaml         # Parámetros cinemáticos del robot (r, b_eff)
├── launch/
│   └── bringup.launch.py         # Lanzador del entorno integrado con Yahboom
├── resource/
│   └── capytown_esan             # Archivo marcador de ROS 2 (Vacío)
├── GUIA.md                       # Documentación de soporte de calibración
├── package.xml                   # Declaración de dependencias (OpenCV, Matplotlib)
├── setup.cfg                     # Configuración de instalación de scripts
└── README.md                     # Documentación general del proyecto (Este archivo)
```

## 2. Requisitos del Sistema y Dependencias

Para ejecutar correctamente este paquete, el entorno debe contar con:

- ROS 2 Humble instalado (o corriendo en el contenedor Docker oficial del laboratorio).
- Librerías del Sistema:
  - python3-opencv (Procesamiento de imágenes y máscaras HSV).
  - python3-matplotlib (Generación de gráficos de telemetría).

Las dependencias se encuentran declaradas formalmente dentro del archivo package.xml.

## 3. Algoritmo de Visión y Control

El guiado del robot se basa en un pipeline estructurado en 4 etapas críticas:

1. Captura y Redimensión: Lectura del flujo de la cámara local a una resolución estándar de 640×480 píxeles.
2. Segmentación HSV: Aplicación de filtros de rango de color para aislar la línea amarilla (borde derecho) y la línea blanca (borde izquierdo).
3. Región de Interés (ROI) y Fusión Lógica: Escaneo horizontal en la fila 360 de la imagen. El algoritmo calcula los centroides de ambos bordes y proyecta el centro virtual del carril, operando de manera robusta incluso si se pierde temporalmente de vista una de las dos líneas.
4. Controlador PD: El error en píxeles (cx_lane - 320) se procesa mediante una ecuación de control Proporcional-Derivativo para suavizar los giros y corregir la trayectoria hacia el centro dinámicamente.

---

## 4. Guía de Ejecución en el Laboratorio

Sigue estos pasos en la terminal del robot Yahboom para compilar y ejecutar los nodos:

### Paso 1: Configurar el Entorno y Compilar
Asegúrate de exportar tu Domain ID en cada terminal y compila el espacio de trabajo:

export ROS_DOMAIN_ID=10  # Cambia por el número de tu grupo
cd /home/pi/yahboomcar_ws
colcon build --packages-select capytown_esan
source install/setup.bash

### Paso 2: Lanzar el Bringup del Robot
En una terminal secundaria, inicializa los sensores por hardware y la odometría base del carro:

export ROS_DOMAIN_ID=10
source /home/pi/yahboomcar_ws/install/setup.bash
ros2 launch capytown_esan bringup.launch.py

### Paso 3: Ejecutar el Seguidor de Carril (Nodo Principal)
En la terminal principal, arranca el script de guiado autónomo con telemetría incorporada:

ros2 run capytown_esan vision_completo

Nota sobre la Telemetría: Al detener el nodo presionando Ctrl + C, el robot frenará inmediatamente por seguridad y emergerá de forma automática una ventana de Matplotlib. Esta mostrará dos gráficas científicas detallando la evolución del error de guiado convertido a centímetros reales y la estimación geométrica de la trayectoria recorrida.

### Ejecución de Prueba (Opcional)
Si deseas realizar pruebas rápidas sin generación de gráficas finales, puedes lanzar el nodo secundario ligero:

ros2 run capytown_esan vision_prueba

---

## 5. Parámetros de Calibración

Cualquier desajuste físico o cambio en la velocidad base o las constantes del control se pueden configurar modificando directamente las variables internas de control en el constructor de los scripts:

- self.base_speed: Velocidad lineal constante (m/s).
- self.kp: Ganancia Proporcional (corrección de desvío rápido).
- self.kd: Ganancia Derivativa (amortiguación de sacudidas/sobreimpulso).
