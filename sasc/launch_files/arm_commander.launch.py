from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("irb6640", package_name="irb6640_ros2_moveit2").to_moveit_configs()


    safety_dist_argument = DeclareLaunchArgument(
        'safety_dist',
        default_value='0.40',
        description="Distance to stop before the target"
    )
    
    arm_commander_node = Node(
        package="sasc",
        executable="arm_commander",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"safety_dist": LaunchConfiguration('safety_dist')}
        ]
    )

    return LaunchDescription([
        safety_dist_argument, #variable argument
        arm_commander_node
    ])