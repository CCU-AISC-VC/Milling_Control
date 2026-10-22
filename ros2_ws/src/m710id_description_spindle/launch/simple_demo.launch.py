# simple_demo.launch.py (Jazzy-correct)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os, yaml

def load_yaml(pkg, relpath):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg = "m710id_description_spindle"

    urdf_xacro = PathJoinSubstitution([FindPackageShare(pkg), "urdf", "m710id_70_with_spindle.urdf.xacro"])
    srdf_path  = PathJoinSubstitution([FindPackageShare(pkg), "config", "m710id_70_spindle.srdf"])
    kinematics = load_yaml(pkg, "config/kinematics.yaml")
    joint_limits = load_yaml(pkg, "config/joint_limits.yaml")
    pilz_limits  = load_yaml(pkg, "config/pilz_cartesian_limits.yaml")
    controllers  = load_yaml(pkg, "config/moveit_controllers.yaml")
    rviz_cfg     = PathJoinSubstitution([FindPackageShare(pkg), "rviz", "moveit.rviz"])

    robot_description = ParameterValue(Command(["xacro ", urdf_xacro]), value_type=str)
    robot_description_semantic = ParameterValue(Command(["cat ", srdf_path]), value_type=str)

    # >>> Jazzy planning-pipelines config (required) <<<
    
    pipelines_path = PathJoinSubstitution([
    FindPackageShare(pkg), "config", "planning_pipelines.ros2.yaml"
        ])

    common_moveit_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics,
        "robot_description_planning": joint_limits,
        "pilz_cartesian_limits": pilz_limits,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": controllers,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[common_moveit_params, pipelines_path],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[common_moveit_params, pipelines_path],
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription([rsp, jsp, move_group, rviz])
