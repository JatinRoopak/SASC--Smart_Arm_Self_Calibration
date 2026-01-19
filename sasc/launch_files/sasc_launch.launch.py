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

    safety_dist_arg = DeclareLaunchArgument(
        'safety_dist',
        default_value='0.40',
        description="initial safety distance"
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
        parameters=[{'safety_dist': LaunchConfiguration('safety_dist')}]
    )

    arm_commander_node = Node(
        package="sasc",
        executable="arm_commander",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time":True},
            {"safety_dist":LaunchConfiguration('safety_dist')}
        ]
    )

    return LaunchDescription([
        safety_dist_arg,
        camera_node,
        error_calibration_node,
        arm_commander_node
    ])