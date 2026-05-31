#!/usr/bin/env python3
#
# gravity_comp_hw.launch.py
# Compensacion gravitacional en hardware real del OpenMANIPULATOR-X.
#
# Uso:
#   ros2 launch open_manipulator_x_torque_control gravity_comp_hw.launch.py
#   ros2 launch open_manipulator_x_torque_control gravity_comp_hw.launch.py port:=/dev/ttyUSB1
#   ros2 launch open_manipulator_x_torque_control gravity_comp_hw.launch.py gravity_scale:=0.95
#
# Argumentos:
#   port          Puerto serial del U2D2       (default: /dev/ttyUSB0)
#   gravity_scale Factor de escala de G(q)     (default: 1.0)
#   deadzone      Deadzone en ticks corriente  (default: 5.0)
#   duration      Duracion en segundos (0=inf) (default: 0.0)
#   log_id        Identificador del CSV        (default: 1)
#
# ADVERTENCIA: Asegurarse de que ningun otro proceso accede a /dev/ttyUSB0
# (ros2_control_node, dynamixel_hardware_interface, etc.) antes de lanzar.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    port          = LaunchConfiguration('port')
    gravity_scale = LaunchConfiguration('gravity_scale')
    deadzone      = LaunchConfiguration('deadzone')
    duration      = LaunchConfiguration('duration')
    log_id        = LaunchConfiguration('log_id')

    params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_torque_control'),
        'config',
        'hw_gravity_comp_params.yaml',
    ])

    gravity_comp_node = Node(
        package='open_manipulator_x_torque_control',
        executable='hw_gravity_comp_node',
        name='hw_gravity_comp_node',
        output='screen',
        parameters=[
            params_file,
            {
                'port_name':     port,
                'gravity_scale': gravity_scale,
                'deadzone_ticks': deadzone,
                'duration_s':    duration,
                'log_id':        log_id,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Puerto serial del adaptador U2D2',
        ),
        DeclareLaunchArgument(
            'gravity_scale',
            default_value='1.0',
            description='Factor de escala de G(q): 1.0=modelo exacto, <1 si cae, >1 si sube',
        ),
        DeclareLaunchArgument(
            'deadzone',
            default_value='5.0',
            description='Compensacion de zona muerta estatica [ticks]',
        ),
        DeclareLaunchArgument(
            'duration',
            default_value='0.0',
            description='Duracion en segundos (0 = indefinido)',
        ),
        DeclareLaunchArgument(
            'log_id',
            default_value='1',
            description='ID del CSV de salida: hw_gc_data_<log_id>.csv',
        ),

        gravity_comp_node,
    ])
