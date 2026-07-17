"""
friction_sweep.launch.py
Lanza hw_sinusoidal_torque_node en modo barrido de VELOCIDAD CONSTANTE para
identificar la fricción articular (analizar con src/Identification/identify_friction.py).

El joint indicado por 'friction_joint' (1..4) se mueve en mode="velocity"; el resto
se mantiene fijo en position. Resuelve la ruta del config automáticamente.
Protocolo completo de identificación (incluye cuándo repetir un barrido):
src/Identification/Ident_OpenManX_XM430W350T_procedure.md

duration_s por defecto es "auto": se calcula como
    t_settle + len(vel_list)·vel_seg_duration + 1 s
leyendo el config, de modo que cambiar vel_seg_duration NUNCA trunca el barrido.
Pasar un número explícito (duration_s:=43.0) desactiva el cálculo.

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

  # Re-grabado con banda angosta y segmentos largos (duration_s se ajusta solo)
  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \
    friction_joint:=2 open_port:=true enable_torque:=true \
    enable_current_commands:=true log_id:=25 vel_band:=0.3 vel_seg_duration:=8.0
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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

    vel_seg = float(LaunchConfiguration('vel_seg_duration').perform(context))

    # duration_s "auto": t_settle + len(vel_list)·vel_seg_duration + margen,
    # para que alargar vel_seg_duration nunca trunque el barrido.
    dur_arg = LaunchConfiguration('duration_s').perform(context)
    actions = []
    if dur_arg.strip().lower() == 'auto':
        with open(params_file) as f:
            cfg = yaml.safe_load(f)['hw_sinusoidal_torque_node']['ros__parameters']
        n_vel    = len(cfg.get('vel_list', [0.0] * 8))
        t_settle = float(cfg.get('t_settle', 2.0))
        duration = t_settle + n_vel * vel_seg + 1.0
        actions.append(LogInfo(msg=f"duration_s=auto → {duration:.1f} s "
                               f"({t_settle:.1f} settle + {n_vel} vel × {vel_seg:.1f} s)"))
    else:
        duration = float(dur_arg)

    overrides = {
        'mode':                    mode,
        'open_port':               as_bool('open_port'),
        'enable_torque':           as_bool('enable_torque'),
        'enable_current_commands': as_bool('enable_current_commands'),
        'log_id':                  int(LaunchConfiguration('log_id').perform(context)),
        'duration_s':              duration,
        'vel_seg_duration':        vel_seg,
        'vel_band':                float(LaunchConfiguration('vel_band').perform(context)),
    }

    actions.append(Node(
        package='open_manipulator_x_torque_control',
        executable='hw_sinusoidal_torque_node',
        name='hw_sinusoidal_torque_node',
        output='screen',
        parameters=[params_file, overrides],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('friction_joint',          default_value='1'),
        DeclareLaunchArgument('open_port',               default_value='false'),
        DeclareLaunchArgument('enable_torque',           default_value='false'),
        DeclareLaunchArgument('enable_current_commands', default_value='false'),
        DeclareLaunchArgument('log_id',                  default_value='20'),
        DeclareLaunchArgument('duration_s',              default_value='auto'),
        DeclareLaunchArgument('vel_seg_duration',        default_value='5.0'),
        DeclareLaunchArgument('vel_band',                default_value='0.5'),
        OpaqueFunction(function=launch_setup),
    ])
