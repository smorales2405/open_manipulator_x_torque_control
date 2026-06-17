#!/usr/bin/env python3
#
# tvlqr_hw.launch.py
# Control TV-LQR en hardware real del OpenMANIPULATOR-X.
#
# Uso:
#   ros2 launch open_manipulator_x_torque_control tvlqr_hw.launch.py
#   ros2 launch open_manipulator_x_torque_control tvlqr_hw.launch.py t_run:=4.0 test_num:=1
#   ros2 launch open_manipulator_x_torque_control tvlqr_hw.launch.py port:=/dev/ttyUSB1 t_warmup:=1.0
#
# Argumentos:
#   port          Puerto serial del U2D2              (default: /dev/ttyUSB0)
#   t_run         Duracion del experimento [s]        (default: 4.0)
#   t_warmup      Duracion del warmup [s]             (default: 0.5)
#   torque_scale  Factor de escala de seguridad [0,1] (default: 1.0)
#   test_num      ID del CSV de salida                (default: 1)
#   vel_cutoff_hz Frecuencia de corte filtro EMA [Hz] (default: 10.0)
#
# ADVERTENCIA: Asegurarse de que ningun otro proceso accede al puerto USB
# (ros2_control_node, dynamixel_hardware_interface) antes de lanzar.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    port          = LaunchConfiguration('port')
    t_run         = LaunchConfiguration('t_run')
    t_warmup      = LaunchConfiguration('t_warmup')
    torque_scale  = LaunchConfiguration('torque_scale')
    test_num      = LaunchConfiguration('test_num')
    vel_cutoff_hz = LaunchConfiguration('vel_cutoff_hz')

    motor_params = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_torque_control'),
        'config',
        'motor_params.yaml',
    ])

    tvlqr_node = Node(
        package='open_manipulator_x_torque_control',
        executable='hw_tvlqr_node',
        name='hw_tvlqr_node',
        output='screen',
        parameters=[
            motor_params,
            {
                'port_name':     port,
                't_run':         t_run,
                't_warmup':      t_warmup,
                'torque_scale':  torque_scale,
                'test_num':      test_num,
                'vel_cutoff_hz': vel_cutoff_hz,
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
            't_run',
            default_value='4.0',
            description='Duracion del experimento en segundos',
        ),
        DeclareLaunchArgument(
            't_warmup',
            default_value='0.0',
            description='Duracion del warmup con compensacion gravitatoria [s]',
        ),
        DeclareLaunchArgument(
            'torque_scale',
            default_value='1.0',
            description='Factor de escala de seguridad para torques [0..1]',
        ),
        DeclareLaunchArgument(
            'test_num',
            default_value='1',
            description='Identificador del CSV de salida',
        ),
        DeclareLaunchArgument(
            'vel_cutoff_hz',
            default_value='0.0',
            description='Frecuencia de corte del filtro EMA de velocidad [Hz]',
        ),

        tvlqr_node,
    ])
