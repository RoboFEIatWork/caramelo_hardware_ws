from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Jazzy warns if ROS_LOCALHOST_ONLY is present, even when it is set to 0.
    os.environ.pop('ROS_LOCALHOST_ONLY', None)

    robot_description_path = get_package_share_path('caramelo_description')
    robot_bringup_path = get_package_share_path('raspberry_bringup')
    localization_path = get_package_share_path('caramelo_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_manipulator = LaunchConfiguration('use_manipulator')
    use_mock_components = LaunchConfiguration('use_mock_components')
    
    urdf_path = os.path.join(robot_description_path, 'urdf', 'robots', 'robot.urdf.xacro')
    mesh_server_port = '8000'
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_manipulator:=', use_manipulator,
            ' use_realsense:=true',
            ' use_mock_components:=', use_mock_components,
            ' visual_mode:=http',
            ' mesh_uri_prefix:=http://raspberrypi.local:', mesh_server_port,
        ]),
        value_type=str,
    )
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'caramelo_controllers.yaml')
    ekf_config = os.path.join(localization_path, 'config', 'ekf.yaml')
    scan_normalizer_config = os.path.join(robot_bringup_path, 'config', 'scan_normalizer.yaml')
    scan_normalizer_script = os.path.join(robot_bringup_path, 'scripts', 'scan_normalizer.py')
    mesh_server_script = os.path.join(robot_bringup_path, 'scripts', 'mesh_server.py')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # SEM prefix de chrt (2026-09-08). O tempo real agora e' aplicado
        # DENTRO do driver, so' nas threads que precisam: o laco de controle e a
        # thread de tx do lgpio (ver <param name="control_rt_priority"> no
        # mobile_base.ros2_control.xacro).
        #
        # POR QUE MUDOU: de 2026-07-29 ate aqui este launch subia o processo
        # inteiro com "chrt -f 50". A intencao era proteger as threads do lgpio
        # ("tx dos pulsos servo + alertas de encoder"), porque sem isso os pulsos
        # ESTICAM sob carga (frente acelera ~2x; provado em bancada 27-29/07).
        # Duas coisas mudaram desde entao:
        #  1. Os "alertas de encoder" nao existem mais — a leitura virou
        #     amostragem do RIO, com thread propria que ja define a sua
        #     prioridade (SCHED_FIFO 80 no core isolado).
        #  2. O prefix elevava as 35 threads do processo, inclusive as NOVE de
        #     DDS. Medido em 2026-09-08 nesta bancada: com o stack do PC no ar, o
        #     ekf_node (prioridade normal) passou de 1 para 21 estouros de
        #     deadline em 2 min, ate 152 ms num ciclo de 25 ms, com os cores
        #     0/1/2 68% OCIOSOS. Nao era falta de CPU: era preempcao por thread
        #     de REDE em prioridade de tempo real. Com
        #     kernel.sched_rt_runtime_us=-1 nao ha teto para isso, e o detector
        #     de lockup deste kernel esta desabilitado ("Hard watchdog
        #     permanently disabled") — um core tomado por thread RT nao gera log
        #     nem se recupera, que e' a assinatura do "a Pi congela".
        #
        # Continua exigindo /etc/security/limits.d/99-realtime.conf (rtprio 98);
        # a diferenca e' que agora, sem o limite, o driver AVISA e segue em
        # prioridade normal em vez de o launch inteiro quebrar.
        parameters=[
            robot_controllers,
            {'use_sim_time': use_sim_time},
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_controller",
            "--controller-ros-args=--remap /mecanum_controller/odometry:=/odom/wheel",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        condition=IfCondition(use_manipulator),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        condition=IfCondition(use_manipulator),
    )

    laser_driver = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_node',
            parameters=[os.path.join(
                robot_bringup_path,
                "config",
                "rplidar_s2.yaml"
            )],
            remappings=[
                ('scan', 'scan_raw'),
            ],
            output="screen",
            condition=IfCondition(use_lidar),
    )

    scan_normalizer = ExecuteProcess(
        cmd=[
            'python3',
            scan_normalizer_script,
            '--ros-args',
            '--params-file',
            scan_normalizer_config,
        ],
        output='screen',
        condition=IfCondition(use_lidar),
    )

    imu_driver = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        parameters=[{
            'port': '/dev/imu_usb',
            'baud': 9600,
            'frame_id': 'imu_link',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(use_imu),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    mesh_server = ExecuteProcess(
        cmd=[
            # nice +15: servir STL e cosmetico (so o RViz usa). Sem isso, o pico
            # de CPU ao abrir o RViz roubava ciclo do EKF na Pi ("Failed to
            # meet update rate" em rajadas, visto 2026-07-21).
            'nice', '-n', '15',
            'python3',
            mesh_server_script,
            '--port',
            mesh_server_port,
            '--host',
            '0.0.0.0',
            '--directory',
            str(robot_description_path),
        ],
        output='screen',
    )

    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[Shutdown(reason='ros2_control_node exited')],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usa clock simulado se true',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Inicia o RPLIDAR e o scan_normalizer se true',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Inicia a IMU WIT se true',
        ),
        DeclareLaunchArgument(
            'use_manipulator',
            default_value='true',
            description='Inclui o manipulador no URDF/ros2_control e sobe arm/gripper '
                        'controllers se true. Use false p/ testes so de base (braco '
                        'desenergizado/desconectado — sem /dev/manip_usb o configure '
                        'do Arm mata o controller_manager inteiro).',
        ),
        DeclareLaunchArgument(
            'use_mock_components',
            default_value='false',
            description='Usa mock_components para o ros2_control do manipulador se true',
        ),
        mesh_server,
        robot_state_publisher_node,
        laser_driver,
        scan_normalizer,
        imu_driver,
        control_node,
        shutdown_on_control_exit,
        joint_state_broadcaster_spawner,
        mecanum_drive_controller_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        ekf_node,
    ])
