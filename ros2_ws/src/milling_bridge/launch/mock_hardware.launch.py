"""ros2_control with mock hardware for the M-710iD/70 + spindle, plus robot_state_publisher.

Uses milling_bridge/urdf/m710id_70_spindle_ros2_control.urdf.xacro (robot macro + the MoveIt
config's ros2_control macro with ``mock_components/GenericSystem``) and the MoveIt config's
ros2_controllers.yaml. No RViz, no MoveIt.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "m710id_70_moveit_config"
    urdf = PathJoinSubstitution([FindPackageShare("milling_bridge"), "urdf", "m710id_70_spindle_ros2_control.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare(pkg), "config", "ros2_controllers.yaml"])
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str)}
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[robot_description, {"use_sim_time": use_sim_time}]),
            Node(package="controller_manager", executable="ros2_control_node", output="screen", parameters=[robot_description, controllers, {"use_sim_time": use_sim_time}]),
            Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], output="screen"),
            Node(package="controller_manager", executable="spawner", arguments=["arm_controller", "--controller-manager", "/controller_manager"], output="screen"),
        ]
    )
