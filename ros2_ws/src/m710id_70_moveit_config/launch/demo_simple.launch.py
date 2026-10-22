#!/usr/bin/env python3
"""
Simple MoveIt demo launch file for M710iD/70 with spindle
Uses the URDF wrapper from MoveIt config package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, relative_path):
    """Load a YAML file and return its contents"""
    package_share = get_package_share_directory(package_name)
    abs_path = os.path.join(package_share, relative_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Warning: Could not load {abs_path}: {e}")
        return {}


def generate_launch_description():
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
    ]
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package name
    moveit_config_pkg = 'm710id_70_moveit_config'
    
    # File paths - using MoveIt config's own wrapper
    urdf_xacro_path = PathJoinSubstitution([
        FindPackageShare(moveit_config_pkg),
        'config',
        'm710id_70_spindle.urdf.xacro'
    ])
    
    srdf_path = PathJoinSubstitution([
        FindPackageShare(moveit_config_pkg),
        'config',
        'm710id_70_spindle.srdf'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(moveit_config_pkg),
        'config',
        'moveit.rviz'
    ])
    
    # Generate robot description from xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str
    )
    
    # Load SRDF
    robot_description_semantic_content = ParameterValue(
        Command(['cat ', srdf_path]),
        value_type=str
    )
    
    # Load configuration files
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    moveit_controllers_yaml = load_yaml(moveit_config_pkg, 'config/moveit_controllers.yaml')
    
    # OMPL planning configuration
    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        }
    }
    
    # Try to load OMPL config
    ompl_config = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    if ompl_config:
        ompl_planning_pipeline_config['ompl'].update(ompl_config)
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }
    
    # Build move_group node parameters
    move_group_params = {
        'robot_description': robot_description_content,
        'robot_description_semantic': robot_description_semantic_content,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'use_sim_time': use_sim_time,
    }
    
    # Add OMPL configuration
    move_group_params.update(ompl_planning_pipeline_config)
    
    # Add planning scene monitor parameters
    move_group_params.update(planning_scene_monitor_parameters)
    
    # Add controller configuration
    move_group_params.update(moveit_controllers_yaml)
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Joint State Publisher Node (for demo/fake mode)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['move_group/fake_controller_joint_states'],
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[move_group_params],
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'robot_description': robot_description_content,
            'robot_description_semantic': robot_description_semantic_content,
            'robot_description_kinematics': kinematics_yaml,
            'robot_description_planning': joint_limits_yaml,
            'use_sim_time': use_sim_time,
        }],
    )
    
    # Static TF for world -> base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )
    
    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            joint_state_publisher_node,
            static_tf_node,
            move_group_node,
            rviz_node,
        ]
    )
