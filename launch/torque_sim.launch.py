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
# Posicion inicial del robot:
#   Controlada por  config/sim_init_config.yaml
#     use_fixed_init: false → equilibrio bajo gravedad sin torques (por defecto)
#     use_fixed_init: true  → posicion articular fija dada por q_init
#
# Notas de migracion Gazebo Classic → Fortress:
#   - gazebo_ros             → ros_gz_sim
#   - gazebo.launch.py       → gz_sim.launch.py
#   - spawn_entity.py        → ros_gz_sim create  (-name en lugar de -entity)
#   - GAZEBO_PLUGIN_PATH     → no se necesita (gz_ros2_control usa ament index)
#   - /clock bridge          → ros_gz_bridge obligatorio con use_sim_time:=true

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
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


def _load_init_config():
    """Lee config/sim_init_config.yaml y devuelve (use_fixed_init_str, q_init, spawn_obs, obs_pose)."""
    pkg_share = get_package_share_directory('open_manipulator_x_torque_control')
    cfg_path  = os.path.join(pkg_share, 'config', 'sim_init_config.yaml')
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)
    use_fixed = 'true' if cfg.get('use_fixed_init', False) else 'false'
    q_init    = cfg.get('q_init', [0.0, 0.0, 0.0, 0.0])
    spawn_obs = cfg.get('spawn_obstacle', False)
    obs_pose  = cfg.get('obstacle_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    return use_fixed, q_init, spawn_obs, obs_pose


def generate_launch_description():

    prefix     = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')

    # Leer configuracion de posicion inicial y obstáculo desde YAML
    use_fixed_init, q_init, spawn_obs, obs_pose = _load_init_config()

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
            ' prefix:=',       prefix,
            ' use_sim:=true',
            ' use_fixed_init:=', use_fixed_init,
            ' q1_init:=',      str(q_init[0]),
            ' q2_init:=',      str(q_init[1]),
            ' q3_init:=',      str(q_init[2]),
            ' q4_init:=',      str(q_init[3]),
        ]),
        value_type=str,
    )

    # Mata procesos gz huerfanos de sesiones anteriores.
    # En Fortress el simulador corre como proceso unico 'gz' (no gzserver/gzclient).
    cleanup_gz = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -9 gz 2>/dev/null || true; '
            'pkill -9 ign 2>/dev/null || true; '
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

    # Spawn del obstáculo MDF (condicional según spawn_obstacle en sim_init_config.yaml)
    spawn_obstacle_node = None
    if spawn_obs:
        _pkg = get_package_share_directory('open_manipulator_x_torque_control')
        _stl = 'file://{}/meshes/obstacle_mdf.stl'.format(_pkg)
        # Colisión: 3 cajas primitivas en vez de la malla STL cóncava.
        # ODE no resuelve bien colisiones de mallas cóncavas (arcos/puentes);
        # las trata como casco convexo y deja "huecos" en las paredes.
        # Dimensiones en marco local del link (antes de aplicar la pose del modelo):
        #   STL [mm]: X∈[-160,+160], Y∈[-75,+75], Z∈[0,158]  → scale 0.001
        #   Techo  : 320×150×3 mm  → centro (0, 0, 0.1565 m)
        #   Pared-I: 320×3×155 mm  → centro (0, -0.0735, 0.0775 m)
        #   Pared-D: 320×3×155 mm  → centro (0, +0.0735, 0.0775 m)
        _col = (
            '<surface><contact><ode><kp>1000000</kp><kd>100</kd></ode></contact>'
            '<friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>'
        )
        _sdf = (
            '<?xml version="1.0"?><sdf version="1.9">'
            '<model name="obstacle_mdf"><static>true</static><link name="link">'
            '<visual name="visual"><geometry><mesh>'
            '<uri>{uri}</uri><scale>0.001 0.001 0.001</scale>'
            '</mesh></geometry>'
            '<material><ambient>0.75 0.60 0.42 1</ambient>'
            '<diffuse>0.75 0.60 0.42 1</diffuse>'
            '<specular>0.08 0.06 0.04 1</specular></material></visual>'
            '<collision name="col_top"><pose>0 0 0.1565 0 0 0</pose>'
            '<geometry><box><size>0.320 0.150 0.003</size></box></geometry>'
            + _col + '</collision>'
            '<collision name="col_left"><pose>0 -0.0735 0.0775 0 0 0</pose>'
            '<geometry><box><size>0.320 0.003 0.155</size></box></geometry>'
            + _col + '</collision>'
            '<collision name="col_right"><pose>0 0.0735 0.0775 0 0 0</pose>'
            '<geometry><box><size>0.320 0.003 0.155</size></box></geometry>'
            + _col + '</collision>'
            '</link></model></sdf>'
        ).format(uri=_stl)
        spawn_obstacle_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'obstacle_mdf',
                '-string', _sdf,
                '-x', str(obs_pose[0]), '-y', str(obs_pose[1]), '-z', str(obs_pose[2]),
                '-R', str(obs_pose[3]), '-P', str(obs_pose[4]), '-Y', str(obs_pose[5]),
            ],
            output='screen',
            name='spawn_obstacle',
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

    # spawn termina → espera 3 s (hardware interface se activa en el 1er paso de sim) → jsb_spawner
    # + spawn del obstáculo MDF si está habilitado en sim_init_config.yaml
    _post_spawn = [TimerAction(period=3.0, actions=[jsb_spawner])]
    if spawn_obstacle_node is not None:
        _post_spawn.append(spawn_obstacle_node)
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=_post_spawn,
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
