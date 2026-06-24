"""
friction_sweep.launch.py
Lanza hw_sinusoidal_torque_node en modo barrido de VELOCIDAD CONSTANTE para
identificar la fricción articular (analizar con src/Identification/identify_friction.py).

El joint indicado por 'friction_joint' (1..4) se mueve en mode="velocity"; el resto
se mantiene fijo en position. Resuelve la ruta del config automáticamente.

Ejemplos:
  # Dry-run (sin hardware) — verifica parámetros
  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \
    friction_joint:=1 log_id:=20

  # Ensayo real de J1 (eje vertical → fricción pura, sin gravedad)
  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \
    friction_joint:=1 open_port:=true enable_torque:=true \
    enable_current_commands:=true log_id:=20

  # Ensayo real de J2 / J3 / J4 (cambiar friction_joint y log_id)
  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \
    friction_joint:=2 open_port:=true enable_torque:=true \
    enable_current_commands:=true log_id:=21
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('open_manipulator_x_torque_control')
    params_file = os.path.join(pkg, 'config', 'hw_friction_sweep_params.yaml')

    fj = int(LaunchConfiguration('friction_joint').perform(context))
    if fj < 1 or fj > 4:
        raise RuntimeError("friction_joint debe estar en 1..4")
    mode = ['position'] * 4
    mode[fj - 1] = 'velocity'

    def as_bool(name):
        return LaunchConfiguration(name).perform(context).lower() == 'true'

    overrides = {
        'mode':                    mode,
        'open_port':               as_bool('open_port'),
        'enable_torque':           as_bool('enable_torque'),
        'enable_current_commands': as_bool('enable_current_commands'),
        'log_id':                  int(LaunchConfiguration('log_id').perform(context)),
        'duration_s':              float(LaunchConfiguration('duration_s').perform(context)),
        'vel_seg_duration':        float(LaunchConfiguration('vel_seg_duration').perform(context)),
        'vel_band':                float(LaunchConfiguration('vel_band').perform(context)),
    }

    return [Node(
        package='open_manipulator_x_torque_control',
        executable='hw_sinusoidal_torque_node',
        name='hw_sinusoidal_torque_node',
        output='screen',
        parameters=[params_file, overrides],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('friction_joint',          default_value='1'),
        DeclareLaunchArgument('open_port',               default_value='false'),
        DeclareLaunchArgument('enable_torque',           default_value='false'),
        DeclareLaunchArgument('enable_current_commands', default_value='false'),
        DeclareLaunchArgument('log_id',                  default_value='20'),
        DeclareLaunchArgument('duration_s',              default_value='43.0'),
        DeclareLaunchArgument('vel_seg_duration',        default_value='5.0'),
        DeclareLaunchArgument('vel_band',                default_value='0.5'),
        OpaqueFunction(function=launch_setup),
    ])
