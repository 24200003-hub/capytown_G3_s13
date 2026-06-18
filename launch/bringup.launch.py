#!/usr/bin/env python3
"""
bringup.launch.py - Arranque del Yahboom MicroROS-Pi5 para RC-1.

QUE HACE:
  - Incluye el bringup oficial de Yahboom (publica /odom, /tf, /scan; suscribe /cmd_vel).
  - Carga los parametros del modelo (wheel_params.yaml), incluido el b_eff calibrado.

NOTA: En el reto, bringup.launch.py lo provee la catedra. Esta es una version de
REFERENCIA que envuelve el bringup de Yahboom (yahboomcar_bringup). Ajusta el
nombre del launch/paquete y los remapeos al robot real del laboratorio.

ANTES DE LANZAR (lecciones de campo de esta plataforma):
  1) export ROS_DOMAIN_ID=<n>   (el numero del grupo) en CADA terminal.
  2) Levanta el agente micro-ROS y presiona el boton RESET del ESP32 para que
     establezca sesion; si no, /odom no publica y los motores no responden.
  3) Verifica: ros2 topic hz /odom  (debe dar ~50 Hz) y que /cmd_vel mueva el robot.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    wheel_params = os.path.join(
        get_package_share_directory('capytown_esan'), 'config', 'wheel_params.yaml')

    # Bringup oficial de Yahboom (ajusta si tu imagen usa otro nombre).
    yahboom_share = get_package_share_directory('yahboomcar_bringup')
    yahboom_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_share, 'launch', 'yahboomcar_bringup_launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('wheel_params', default_value=wheel_params,
                              description='YAML con b_eff y demas parametros del modelo'),
        yahboom_bringup,
    ])
