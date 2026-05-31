#!/usr/bin/env python3
#
# gravity_comp_gz.launch.py
# Simulacion de compensacion gravitacional en Gazebo Fortress.
#
# Diferencia clave respecto a torque_sim.launch.py:
#   joint_state_broadcaster + arm_effort_controller + gripper_controller
#   se lanzan en PARALELO (no secuencialmente) para minimizar el tiempo
#   en que el robot cae sin torque aplicado.
#
# El nodo gz_gravity_comp_node aplica:
#   tau = gravity_scale * G(q) + Kp*(q_des - q) + Kd*(0 - dq)
#
# El termino PD recupera el robot de la caida inicial (inevitable por la
# latencia de Gazebo) y lo mantiene en q_des = [0,0,0,0].
#
# Uso:
#   ros2 launch open_manipulator_x_torque_control gravity_comp_gz.launch.py
#   ros2 launch open_manipulator_x_torque_control gravity_comp_gz.launch.py Kp:=50.0
#   ros2 launch open_manipulator_x_torque_control gravity_comp_gz.launch.py t_sim:=30.0

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
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

from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    gravity_scale = LaunchConfiguration('gravity_scale')
    Kp            = LaunchConfiguration('Kp')
    Kd            = LaunchConfiguration('Kd')
    test_num      = LaunchConfiguration('test_num')
    t_sim         = LaunchConfiguration('t_sim')

    pkg_share = FindPackageShare('open_manipulator_x_torque_control')

    world = PathJoinSubstitution([
        pkg_share, 'worlds', 'empty_world_fortress.sdf',
    ])

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([pkg_share, 'scripts', 'xacro_oneline.sh']),
            ' ',
            PathJoinSubstitution([
                pkg_share, 'xacro',
                'open_manipulator_x_effort_robot.urdf.xacro',
            ]),
            ' prefix:= use_sim:=true',
        ]),
        value_type=str,
    )

    # ── 1. Limpiar procesos gz huerfanos ──────────────────────────────────────
    cleanup_gz = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -9 gz 2>/dev/null || true; '
            'pkill -9 ign 2>/dev/null || true; '
            'sleep 1; echo "[cleanup] listo"',
        ],
        output='screen',
        name='cleanup_gz',
    )

    # ── 2. Robot state publisher ───────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── 3. Gazebo Fortress ─────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── 4. Bridge de /clock (obligatorio para use_sim_time) ───────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        name='clock_bridge',
    )

    # ── 5. Spawn del robot en Gazebo ──────────────────────────────────────────
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

    # ── 6. Controladores — arrancan EN PARALELO tras 3 s del spawn ────────────
    #   Reducir la ventana de caida libre respecto a torque_sim.launch.py
    #   (donde effort_controller espera a que jsb termine completamente).
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

    # ── 7. Nodo de compensacion gravitacional ─────────────────────────────────
    #   Arranca junto con los controladores; su guard interna
    #   (if !last_js_ return) lo mantiene inactivo hasta recibir /joint_states.
    gz_gravity_comp = Node(
        package='open_manipulator_x_torque_control',
        executable='gz_gravity_comp_node',
        name='gz_gravity_comp_node',
        output='screen',
        parameters=[{
            'gravity_scale': gravity_scale,
            'Kp':            Kp,
            'Kd':            Kd,
            'test_num':      test_num,
            't_sim':         t_sim,
        }],
    )

    # ── Secuencia de eventos ──────────────────────────────────────────────────
    # cleanup → RSP + Gazebo + clock + spawn (juntos)
    start_after_cleanup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cleanup_gz,
            on_exit=[robot_state_pub, gz_sim, clock_bridge, spawn_robot],
        )
    )

    # spawn termina → espera 3 s → jsb + effort + gripper + gravity_comp
    # TODOS en paralelo para minimizar el tiempo de caida libre.
    all_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        jsb_spawner,
                        effort_spawner,
                        gripper_spawner,
                        gz_gravity_comp,
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gravity_scale',
            default_value='1.0',
            description='Factor de escala de G(q). <1 si cae, >1 si sube',
        ),
        DeclareLaunchArgument(
            'Kp',
            default_value='3.0',
            description='Ganancia proporcional PD. Kp_max≈3 para joint2 con budget=0.475 N.m',
        ),
        DeclareLaunchArgument(
            'Kd',
            default_value='0.5',
            description='Ganancia derivativa PD (amortiguamiento)',
        ),
        DeclareLaunchArgument(
            'test_num',
            default_value='1',
            description='ID del CSV: gz_gc_data_<test_num>.csv',
        ),
        DeclareLaunchArgument(
            't_sim',
            default_value='0.0',
            description='Duracion en segundos (0 = ilimitado)',
        ),

        cleanup_gz,
        start_after_cleanup,
        all_after_spawn,
    ])
