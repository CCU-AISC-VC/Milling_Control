# milling_bridge — 軌跡檔 → FollowJointTrajectory

讀 `docs/SPEC.md` §5 的軌跡資料夾（`trajectory.csv` + `meta.json`），送到 `arm_controller/follow_joint_trajectory`（真機：`fanuc_ros2_driver`；測試：`mock_hardware.launch.py` 的 ros2_control 假硬體），同時把 `/joint_states` 錄成 CSV。

| 節點 | 參數（預設） |
|---|---|
| `trajectory_player` | `trajectory_dir`、`dry_run`（**true**）、`speed_scale`（**0.1**；> 1 要 `allow_fast:=true`，只給模擬用）、`action_name`、`move_to_start`（true，先用 ≤ 0.2 rad/s 走到第一點）、`lead_in`（0.5 s）、`goal_time_tolerance`（0.5 s） |
| `joint_state_recorder` | `output_dir`、`joints`（J1–J6）；寫 `joint_states.csv`、`events.json`（player 送出軌跡的時間戳） |

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
# 假硬體
ros2 launch milling_bridge mock_hardware.launch.py
# 乾跑（只印摘要）
ros2 launch milling_bridge play_trajectory.launch.py trajectory_dir:=$PWD/outputs/m2_ik/sample_pocket_contour/trajectory output_dir:=$PWD/outputs/m4_bridge/dry
# 真的送（假硬體可用 allow_fast）
ros2 launch milling_bridge play_trajectory.launch.py trajectory_dir:=... output_dir:=... dry_run:=false speed_scale:=0.2
```

⚠️ 真機：主軸 TCP 與工件位置未實測前（SPEC §3.2），只在空中執行、不裝刀、不接觸工件。
