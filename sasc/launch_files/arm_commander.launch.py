from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("irb6640", package_name="irb6640_ros2_moveit2").to_moveit_configs()

    arm_commander_node = Node(
        package="sasc",
        executable="arm_commander",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        arm_commander_node
    ])