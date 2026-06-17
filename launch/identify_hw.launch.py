"""
identify_hw.launch.py
Lanza hw_identify_node para identificación de la conversión torque→corriente.

Todos los parámetros de seguridad son false/0.0 por defecto.
Ejemplo de uso progresivo:

  # Fase 0 — dry_run (sin hardware)
  ros2 launch open_manipulator_x_torque_control identify_hw.launch.py \
    mode:=dry_run trajectory_scale:=1.0 current_scale:=1.0 log_id:=1

  # Fase 1 — read_only (lectura sin corriente)
  ros2 launch open_manipulator_x_torque_control identify_hw.launch.py \
    mode:=read_only open_port:=true log_id:=1

  # Fase 2 — zero_current (verificar canal de corriente)
  ros2 launch open_manipulator_x_torque_control identify_hw.launch.py \
    mode:=zero_current open_port:=true enable_torque:=true \
    enable_current_commands:=true log_id:=1

  # Fase 3 — smooth_excitation (excitación suave, sin enviar corriente)
  ros2 launch open_manipulator_x_torque_control identify_hw.launch.py \
    mode:=smooth_excitation open_port:=true enable_torque:=true \
    trajectory_scale:=0.3 current_scale:=0.0 log_id:=1

  # Fase 3b — smooth_excitation con corriente (escala baja)
  ros2 launch open_manipulator_x_torque_control identify_hw.launch.py \
    mode:=smooth_excitation open_port:=true enable_torque:=true \
    enable_current_commands:=true trajectory_scale:=0.3 current_scale:=0.3 log_id:=2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('open_manipulator_x_torque_control')
    params_file = os.path.join(pkg, 'config', 'hw_identify_params.yaml')

    return LaunchDescription([
        # ── Argumentos de seguridad ──────────────────────────────────────────
        DeclareLaunchArgument('mode',                    default_value='dry_run'),
        DeclareLaunchArgument('open_port',               default_value='false'),
        DeclareLaunchArgument('enable_torque',           default_value='false'),
        DeclareLaunchArgument('enable_current_commands', default_value='false'),
        DeclareLaunchArgument('current_scale',           default_value='0.0'),
        DeclareLaunchArgument('trajectory_scale',        default_value='0.0'),

        # ── Argumentos de sesión ─────────────────────────────────────────────
        DeclareLaunchArgument('log_id',     default_value='1'),
        DeclareLaunchArgument('duration_s', default_value='30.0'),
        DeclareLaunchArgument('t_ramp',     default_value='3.0'),

        # ── Argumentos de friction_test ──────────────────────────────────────
        DeclareLaunchArgument('friction_joint', default_value='1'),
        DeclareLaunchArgument('friction_w',     default_value='0.1'),
        DeclareLaunchArgument('friction_A',     default_value='0.3'),

        # ── Nodo ─────────────────────────────────────────────────────────────
        Node(
            package='open_manipulator_x_torque_control',
            executable='hw_identify_node',
            name='hw_identify_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'mode':                    LaunchConfiguration('mode'),
                    'open_port':               LaunchConfiguration('open_port'),
                    'enable_torque':           LaunchConfiguration('enable_torque'),
                    'enable_current_commands': LaunchConfiguration('enable_current_commands'),
                    'current_scale':           LaunchConfiguration('current_scale'),
                    'trajectory_scale':        LaunchConfiguration('trajectory_scale'),
                    'log_id':                  LaunchConfiguration('log_id'),
                    'duration_s':              LaunchConfiguration('duration_s'),
                    't_ramp':                  LaunchConfiguration('t_ramp'),
                    'friction_joint':          LaunchConfiguration('friction_joint'),
                    'friction_w':              LaunchConfiguration('friction_w'),
                    'friction_A':              LaunchConfiguration('friction_A'),
                }
            ],
        ),
    ])
