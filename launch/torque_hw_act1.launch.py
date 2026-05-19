#!/usr/bin/env python3
#
# torque_hw_act1.launch.py
# Lanza el control articular FL/Computed-Torque sobre hardware real.
# Usa Dynamixel SDK directo — NO compatible con hardware.launch.py.
#
# Uso:
#   ros2 launch open_manipulator_x_torque_control torque_hw_act1.launch.py
#   ros2 launch ... torque_hw_act1.launch.py gain_scale:=0.5 log_id:=2
#
# Argumentos sobreescribibles desde CLI:
#   gain_scale      — escala de ganancias (1.0 = base)
#   deadzone_ticks  — compensación zona muerta [ticks]
#   viscous_comp    — compensación viscosa
#   duration_s      — duración del experimento [s]
#   log_id          — identificador del CSV de salida
#   port_name       — puerto serie del U2D2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = 'open_manipulator_x_torque_control'


def generate_launch_description():

    params_file = PathJoinSubstitution([
        FindPackageShare(PKG), 'config', 'hw_control_params.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument('port_name',      default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gain_scale',     default_value='1.0'),
        DeclareLaunchArgument('deadzone_ticks', default_value='0.0'),
        DeclareLaunchArgument('viscous_comp',   default_value='0.0'),
        DeclareLaunchArgument('duration_s',     default_value='25.0'),
        DeclareLaunchArgument('log_id',         default_value='1',
                              description='Identificador del CSV hw_fl_data_<log_id>.csv'),

        Node(
            package=PKG,
            executable='hw_fl_control_node',
            name='hw_fl_control_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'port_name':      LaunchConfiguration('port_name'),
                    'gain_scale':     LaunchConfiguration('gain_scale'),
                    'deadzone_ticks': LaunchConfiguration('deadzone_ticks'),
                    'viscous_comp':   LaunchConfiguration('viscous_comp'),
                    'duration_s':     LaunchConfiguration('duration_s'),
                    'log_id':         LaunchConfiguration('log_id'),
                },
            ],
        ),
    ])
