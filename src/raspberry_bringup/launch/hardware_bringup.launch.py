from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_description_path = get_package_share_path('caramelo_description')
    robot_bringup_path = get_package_share_path('raspberry_bringup')
    
    urdf_path = os.path.join(robot_description_path, 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz') # para a raspberry o rviz nao funciona, entao nao tem problema deixar o caminho aqui, mas ele nao vai ser usado
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'caramelo_controllers.yaml')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
    )    
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        mecanum_drive_controller_spawner,
        rviz_node,
    ])
