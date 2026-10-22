# src/m710id_description_spindle/launch/rviz_model_test.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = 'm710id_description_spindle'
    urdf_xacro = PathJoinSubstitution([FindPackageShare(pkg), 'urdf', 'm710id_70_with_spindle.urdf.xacro'])
    rviz_cfg   = PathJoinSubstitution([FindPackageShare(pkg), 'rviz', 'robot_view.rviz'])

    robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', parameters=[{'robot_description': robot_description}], output='screen')

    jsp = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
               name='joint_state_publisher_gui', output='screen')

    rviz = Node(package='rviz2', executable='rviz2',
                name='rviz2', output='screen',
                parameters=[{'robot_description': robot_description}],
                arguments=['-d', rviz_cfg])

    return LaunchDescription([rsp, jsp, rviz])
