#!/usr/bin/env python3
"""
fkin_rviz.launch.py
===================
Lanza el OpenManipulator-X en RViz con:
  · joint_state_publisher_gui  — sliders para mover las articulaciones
  · robot_state_publisher       — publica TF desde el URDF
  · fkin_display_node           — imprime FK en terminal, publica markers y TF de yf
  · rviz2                       — visualizacion con config fkin_rviz.rviz

Uso:
  ros2 launch open_manipulator_x_torque_control fkin_rviz.launch.py

Para cambiar el target yf edita config/fkin_params.yaml y relanza.

Notas URDF:
  Se usa open_manipulator_x_description/urdf/open_manipulator_x_robot.urdf.xacro
  (URDF oficial ROBOTIS con meshes correctas).  El script xacro_oneline.sh elimina
  comentarios y newlines para evitar errores de yaml-cpp en robot_state_publisher.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg         = get_package_share_directory('open_manipulator_x_torque_control')
    rviz_path   = os.path.join(pkg, 'config', 'fkin_rviz.rviz')
    params_path = os.path.join(pkg, 'config', 'fkin_params.yaml')

    # ── URDF: xacro oficial de open_manipulator_x_description ─────────────
    # xacro_oneline.sh: elimina comentarios XML y newlines (evita errores yaml-cpp).
    # Argumentos: use_fake_hardware=true → no requiere hardware real ni Gazebo.
    xacro_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_description'),
        'urdf',
        'open_manipulator_x_robot.urdf.xacro',
    ])
    xacro_oneline = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_torque_control'),
        'scripts',
        'xacro_oneline.sh',
    ])

    robot_description = ParameterValue(
        Command([
            xacro_oneline, ' ',
            xacro_file,
            ' prefix:=""',
            ' use_sim:=false',
            ' use_fake_hardware:=true',
            ' fake_sensor_commands:=true',
            ' port_name:=/dev/ttyUSB0',
        ]),
        value_type=str)

    # ── robot_state_publisher ──────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ── joint_state_publisher_gui (sliders) ───────────────────────────────
    # Los sliders arrancan en la posicion x0 del Lab 5.
    # Nombres de joints del URDF oficial: joint1-4, gripper_left_joint, gripper_right_joint
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'zeros': {
                'joint1':             1.5708,   # pi/2  (x0 lab5)
                'joint2':             0.0,
                'joint3':             0.5236,   # pi/6
                'joint4':             1.0472,   # pi/3
                'gripper_left_joint': 0.0,
                'gripper_right_joint': 0.0,
            },
        }],
    )

    # ── fkin_display_node ─────────────────────────────────────────────────
    # Parametros cargados desde config/fkin_params.yaml.
    # Para cambiar yf edita ese archivo y relanza.
    fkin_node = Node(
        package='open_manipulator_x_torque_control',
        executable='fkin_display_node.py',
        name='fkin_display',
        output='screen',
        parameters=[params_path],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        fkin_node,
        rviz_node,
    ])
