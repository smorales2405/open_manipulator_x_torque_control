#!/usr/bin/env python3
#
# Launch para simulacion Gazebo con control de esfuerzo/torque.
#
# Orden de arranque:
#   1. cleanup_gazebo   → mata gzserver/gzclient huerfanos de sesiones previas
#   2. [OnProcessExit cleanup] → robot_state_publisher + gazebo (gzserver+gzclient)
#                                + spawn_entity
#   3. [OnProcessExit spawn]  → joint_state_broadcaster spawner
#   4. [OnProcessExit jsb]    → arm_effort_controller + gripper_controller
#
# GAZEBO_PLUGIN_PATH fix:
#   gazebo_ros2_control/package.xml tiene <export></export> vacio, por lo que
#   GazeboRosPaths.get_paths() no devuelve /opt/ros/humble/lib.
#   gzserver.launch.py SI appenda os.environ['GAZEBO_PLUGIN_PATH'] al env que
#   pasa a gzserver (ver gzserver.launch.py lineas 60-62), por lo que basta con
#   setear os.environ ANTES de que se evaluen los includes.
#
# rcl --param robot_description fix:
#   gazebo_ros2_control 0.4.x pasa el URDF como --param robot_description:=<xml>
#   a rclcpp::NodeOptions; rcl usa yaml-cpp para parsear el valor, que falla si:
#     (a) el string contiene saltos de linea
#     (b) hay ': ' (colon-espacio) en comentarios XML
#   xacro_oneline.sh elimina comentarios y saltos de linea del output de xacro.

import os
from os import environ
from os.path import pathsep

_ROS_LIB = '/opt/ros/humble/lib'
if _ROS_LIB not in environ.get('GAZEBO_PLUGIN_PATH', ''):
    os.environ['GAZEBO_PLUGIN_PATH'] = (
        _ROS_LIB + pathsep + environ.get('GAZEBO_PLUGIN_PATH', '')
    ).rstrip(pathsep)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    prefix     = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')

    world = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_bringup'),
        'worlds',
        'empty_world.model',
    ])

    # Command uses shlex.split (no shell), so pipes don't work inline.
    # xacro_oneline.sh: strips XML comments and newlines so the URDF is a
    # single comment-free line.  Both are required for rcl yaml-cpp parsing.
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_x_torque_control'),
                'scripts',
                'xacro_oneline.sh',
            ]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_x_torque_control'),
                'xacro',
                'open_manipulator_x_effort_robot.urdf.xacro',
            ]),
            ' prefix:=',
            prefix,
            ' use_sim:=true',
        ]),
        value_type=str,
    )

    # Pattern [g]zserver avoids pkill matching its own bash process:
    # the cmdline literal "[g]zserver" does not contain "gzserver" as substring.
    cleanup_gazebo = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -9 -f "[g]zserver" 2>/dev/null; '
            'pkill -9 -f "[g]zclient" 2>/dev/null; '
            'sleep 1; '
            'echo "[cleanup] Gazebo processes cleared"',
        ],
        output='screen',
        name='cleanup_gazebo',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
            'world': world,
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'open_manipulator_x_effort',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_effort_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_bringup'),
        'rviz',
        'open_manipulator_x.rviz',
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    # cleanup → RSP + Gazebo (gzserver+gzclient) + spawn_entity juntos
    start_after_cleanup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cleanup_gazebo,
            on_exit=[robot_state_pub, gazebo, spawn_entity],
        )
    )

    # spawn_entity termina → jsb_spawner
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner],
        )
    )

    # jsb_spawner termina → effort + gripper
    controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[effort_spawner, gripper_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefijo para nombres de joints y links',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Lanzar RViz2 para visualizacion',
        ),

        cleanup_gazebo,
        start_after_cleanup,
        jsb_after_spawn,
        controllers_after_jsb,
        rviz_node,
    ])
