source /opt/ros/jazzy/setup.bash

source install/setup.bash

for show
ros2 launch m710id_description_spindle rviz_model_test.launch.py

for spindle MoveIt demo mode
ros2 launch m710id_70_moveit_config demo_spindle_complete.launch.py

For actual control
ros2 launch m710id_70_moveit_config demo_spindle_real_robot.launch.py demo_mode:=false

# 1) Source ROS 2 + your Fanuc overlay
source /opt/ros/jazzy/setup.bash
source /home/vincent/fanuc_ros2_driver_v1_1_5/install_jazzy_noble/install/setup.bash
ros2 launch fanuc_ros2_driver fanuc_interface.launch.py robot_ip:="192.168.50.81" robot_type:="M-710iD/70"


