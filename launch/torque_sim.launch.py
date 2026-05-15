#!/usr/bin/env python3
#
# Launch para simulacion Gazebo Fortress con control de esfuerzo/torque.
#
# Orden de arranque:
#   1. cleanup_gz       → mata procesos gz huerfanos de sesiones previas
#   2. [OnProcessExit cleanup] → robot_state_publisher + gz_sim + clock_bridge
#                                + spawn (ros_gz_sim create)
#   3. [OnProcessExit spawn]  → joint_state_broadcaster spawner
#   4. [OnProcessExit jsb]    → arm_effort_controller + gripper_controller
#
# Notas de migracion Gazebo Classic → Fortress:
#   - gazebo_ros             → ros_gz_sim
#   - gazebo.launch.py       → gz_sim.launch.py
#   - spawn_entity.py        → ros_gz_sim create  (-name en lugar de -entity)
#   - GAZEBO_PLUGIN_PATH     → no se necesita (gz_ros2_control usa ament index)
#   - /clock bridge          → ros_gz_bridge obligatorio con use_sim_time:=true

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
        FindPackageShare('open_manipulator_x_torque_control'),
        'worlds',
        'empty_world_fortress.sdf',
    ])

    # xacro_oneline.sh: elimina comentarios XML y saltos de linea del URDF.
    # Sigue siendo util porque robot_state_publisher recibe robot_description
    # como parametro ROS (parseado por yaml-cpp), que falla con newlines y
    # con ': ' dentro de comentarios XML.
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

    # Mata procesos gz huerfanos de sesiones anteriores.
    # En Fortress el simulador corre como proceso unico 'gz' (no gzserver/gzclient).
    cleanup_gz = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -9 -f "[g]z sim" 2>/dev/null; '
            'pkill -9 -f "ruby.*gz" 2>/dev/null; '
            'sleep 1; '
            'echo "[cleanup] gz processes cleared"',
        ],
        output='screen',
        name='cleanup_gz',
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

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Bridge de /clock: obligatorio para que use_sim_time funcione en ROS 2.
    # gz_ros2_control no publica /clock automaticamente; ros_gz_bridge si.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        name='clock_bridge',
    )

    # Gazebo Fortress: spawn via ros_gz_sim create
    # Usa -name (no -entity) y lee robot_description del topic /robot_description.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'open_manipulator_x_effort',
            '-topic', 'robot_description',
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

    # cleanup → RSP + gz_sim + clock_bridge + spawn juntos
    start_after_cleanup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cleanup_gz,
            on_exit=[robot_state_pub, gz_sim, clock_bridge, spawn_robot],
        )
    )

    # spawn termina → jsb_spawner
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
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

        cleanup_gz,
        start_after_cleanup,
        jsb_after_spawn,
        controllers_after_jsb,
        rviz_node,
    ])
