from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("irb6640", package_name="irb6640_ros2_moveit2").to_moveit_configs()

    offset_x_arg = DeclareLaunchArgument(
        'offset_x', default_value='0.40', description='Traget X distance (depth)'
    )
    offset_y_arg = DeclareLaunchArgument(
        'offset_y', default_value='0.00', description='Traget Y distance (Horizontal)'
    )
    offset_z_arg = DeclareLaunchArgument(
        'offset_z', default_value='0.00', description='Traget Z distance (Vertical)'
    )

    camera_node = Node(
        package="sasc",
        executable="camera_reader",
        output="screen"
    )

    error_calibration_node = Node(
        package="sasc",
        executable="error_calibration",
        output="screen",
        parameters=[{
            'safety_distnace_x': LaunchConfiguration('offset_x'),
            'safety_distnace_y': LaunchConfiguration('offset_y'),
            'safety_distnace_z': LaunchConfiguration('offset_z')
        }]
    )

    arm_commander_node = Node(
        package="sasc",
        executable="arm_commander",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time":True},
            {"offset_x": LaunchConfiguration('offset_x')},
            {"offset_y": LaunchConfiguration('offset_y')},
            {"offset_z": LaunchConfiguration('offset_z')}
        ]
    )

    return LaunchDescription([
        offset_x_arg,
        offset_y_arg,
        offset_z_arg,
        camera_node,
        error_calibration_node,
        arm_commander_node
    ])