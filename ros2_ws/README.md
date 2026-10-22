# ros2_ws — ROS 2 Jazzy 端（RViz、MoveIt、真機）

實驗室既有的兩個套件原封不動搬到這裡（2026-10-22，原本在 `FANUC_Spindle_Modified/src/`）：

| 套件 | 內容 |
|---|---|
| `m710id_description_spindle` | M-710iD/70 ＋ 主軸的 xacro、網格、RViz 設定、`rviz_model_test.launch.py`、`moveit_demo.launch.py` |
| `m710id_70_moveit_config` | MoveIt 設定（SRDF、kinematics、controllers、`mock_components/GenericSystem` 假硬體）；`demo_spindle_complete.launch.py`、`demo_spindle_real_robot.launch.py` |
| `scripts_legacy/` | 原本的兩個測試腳本（`simple_test_spindle.py` 用 ROS 1 的 `moveit_commander`，在 ROS 2 跑不起來；`test_spindle_tcp.py` 用 MoveGroup action） |

M4 會新增 `milling_bridge`：讀 `docs/SPEC.md` §5 的軌跡檔，送 `follow_joint_trajectory`。

## 建置

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

`build/`、`install/`、`log/` 不進 git。

## 原本的使用方式（`README_original.txt`）

```bash
# 只看模型
ros2 launch m710id_description_spindle rviz_model_test.launch.py

# MoveIt demo（假硬體）
ros2 launch m710id_70_moveit_config demo_spindle_complete.launch.py

# 真機（在接 FANUC 的電腦上；先啟動 FANUC driver）
source /home/vincent/fanuc_ros2_driver_v1_1_5/install_jazzy_noble/install/setup.bash
ros2 launch fanuc_ros2_driver fanuc_interface.launch.py robot_ip:="192.168.50.81" robot_type:="M-710iD/70"
ros2 launch m710id_70_moveit_config demo_spindle_real_robot.launch.py demo_mode:=false
```

⚠️ 真機動作預設 dry-run、低速。主軸 TCP 偏移尚未實測（`docs/SPEC.md` §3.2），確認前只在空中動作、不接觸工件。
