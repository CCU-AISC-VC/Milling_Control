"""Record joint states and play a trajectory folder (dry-run by default).

    ros2 launch milling_bridge play_trajectory.launch.py trajectory_dir:=/abs/path/trajectory \
        output_dir:=/abs/path/session dry_run:=false speed_scale:=0.2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("trajectory_dir"),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("speed_scale", default_value="0.1"),
        DeclareLaunchArgument("allow_fast", default_value="false"),
        DeclareLaunchArgument("action_name", default_value="/arm_controller/follow_joint_trajectory"),
    ]
    recorder = Node(package="milling_bridge", executable="joint_state_recorder", output="screen", parameters=[{"output_dir": LaunchConfiguration("output_dir")}])
    player = Node(
        package="milling_bridge",
        executable="trajectory_player",
        output="screen",
        parameters=[
            {
                "trajectory_dir": LaunchConfiguration("trajectory_dir"),
                "dry_run": LaunchConfiguration("dry_run"),
                "speed_scale": LaunchConfiguration("speed_scale"),
                "allow_fast": LaunchConfiguration("allow_fast"),
                "action_name": LaunchConfiguration("action_name"),
            }
        ],
    )
    return LaunchDescription(args + [recorder, player])
