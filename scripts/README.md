# scripts/：各階段的執行腳本

從專案根目錄執行：`~/envs/aisc_sim/bin/python scripts/<檔名>.py`。一個腳本對應路線圖的一個階段；共用邏輯在 `src/milling_sim/`，這裡只留流程與參數。每個腳本開頭 `import _bootstrap` 讓 `milling_sim` 不用安裝就能 import。

| 腳本 | 階段 | 用途 | 輸出 |
|---|---|---|---|
| `m0_flatten_urdf.py` | M0 | 用 xacro 攤平 `ros2_ws/src/m710id_description_spindle` 的 URDF，`package://` 改相對路徑，複製網格 | `assets/fanuc_m710id70_spindle/m710id_70_spindle.urdf`、`manifest.json`、`meshes/` |
| `m0_fk_check.py` | M0 | Newton `add_urdf` 與獨立 NumPy FK 對照（`--n` 樣本數、`--seed`）；`home`／`ready` 姿態快照，畫 TCP 座標軸（紅 +X、綠 +Y、藍 +Z） | `outputs/m0_assets/`：`fk_check.csv`、`fk_error_hist.png`、`tcp_samples.png`、`snapshot_*.png`、`summary.json` |
| `m1_make_sample_gcode.py` | M1 | 寫出自製範例 G-code | `assets/gcode/sample_pocket_contour.nc`、`unit_square_100mm.nc` |
| `m1_parse_and_sample.py` | M1 | 解析、取樣（`--max-spacing-mm`、`--chord-tol-mm`、`--accel`、`--corner-deg`、`--rapid-feed`）、畫圖 | `outputs/m1_gcode/<stem>/`：`path.png`、`speed_profile.png`、`samples.csv`、`summary.json` |
| `m2_ik_trajectory.py` | M2 | G-code → 取樣 → cell 變換 → 5 自由度 IK → 檢查 → 軌跡檔（`--gcode`、`--cell`、`--robot`、取樣參數、`--seed`、`--no-newton-check`） | `outputs/m2_ik/<stem>/`：`joints.png`、`joint_velocity.png`、`ik_quality.png`、`tcp_path_base.png`、`summary.json`、`trajectory/{trajectory.csv,meta.json}` |
| `m3_sim_replay.py` | M3 | 軌跡檔在 Newton cell 重播（`--traj`、`--sim-hz`、`--video-speed`、`--no-video`、`--no-usd`）：TCP 追蹤、碰撞判定、USD、GL 影片 | `outputs/m3_sim/<stem>/`：`replay.csv`、`tracking_clearance.png`、`replay_{overview,closeup}.{mp4,gif}`、`frame_*.png`、`scene.usd`、`summary.json` |
| `m3_render_isaacsim.py` | M3 | Isaac Sim 6.1 RTX 渲染 USD（`--usd`、`--frames`、`--subframes`、`--closeup`、`--orbit-deg`）；需 `OMNI_KIT_ACCEPT_EULA=YES` | `outputs/m3_sim/<stem>/rtx*/`：`frames/rgb_*.png`、`render.mp4`、`render_mid.png`、`render_info.json` |
| `m4_run_mock_test.sh` | M4 | 起 ros2_control 假硬體 → 錄 `/joint_states` → 播放軌跡 → 收尾（`<traj_dir> <session_dir> [speed_scale]`；需 ROS 2 Jazzy 與 `colcon build`） | `outputs/m4_bridge/<session>/`：`joint_states.csv`、`events.json`、`player.log`、`mock_hardware.log` |
| `m4_compare_sim_vs_ros.py` | M4 | 錄到的關節曲線 vs 軌跡檔（`--session`、`--traj`、`--label`） | `compare.png`、`compare_summary.json` |
| `m6_dual_replay.py` | M6 | 多台機器人共用時間軸重播（`--traj robot=dir` 可重複、`--cell`、`--views`、`--video-speed`）：追蹤、機器人間淨空（Warp mesh 查詢）、碰撞分類、三視角 GL 影片、USD | `outputs/m6_dual/`：`replay.csv`、`tracking_clearance.png`、`replay_{overview,closeup,side}.{mp4,gif}`、`frame_*.png`、`scene.usd`、`summary.json` |

`m3_render_isaacsim.py --cell <cell.yaml> --camera <name>`：用 cell 設定重畫圍籬／地板／控制櫃（USD 材質、護板半透明）並用設定檔的相機。切削中的工件（USD 的變形網格）會先轉成快照 prim 存成 `scene_render.usd` 再逐幀切換可見度（Kit 不吃變形網格的時間取樣）。`m3_sim_replay.py`／`m6_dual_replay.py` 預設把工件畫成被切的高度圖（`--no-machined-stock` 關閉）。
| `m7_material_removal.py` | M7 | 高度圖材料移除、MRR、切削力、靜態偏移（`--traj robot=dir` 可重複、`--cell`、`--tool`、`--material`、`--resolution-mm`） | `outputs/m7_removal/<name>/`：`heightmap.png`、`surface_3d.png`、`mrr_force_deflection.png`、`removal.csv`、`heightmap_depth_mm.npy`、`summary.json` |
| `m9_dynamics.py` | M9 | Newton 動力學：關節 PD、重力、重力前饋、切削外力（`--traj`、`--modes pd,pd_gff,pd_gff_cut`、`--duration`、`--sim-dt`、`--control-hz`） | `outputs/m9_dynamics/<stem>/`：`dyn_<mode>.csv`、`tip_error_torque.png`、`joint_errors.png`、`summary.json` |
| `make_logo_gcode.py` | 示範 | 圖片（PNG）→ 三軸雕刻 G-code：暗色區域的輪廓 ＋ 填充掃描線（`--width-mm`、`--depth-mm`、`--tool-mm`、`--threshold`、`--preview`） | `assets/gcode/<name>.nc`、預覽 PNG |
| `m8_ui.py` | M8 | 學生網頁 UI（viser，`--port`） | 瀏覽器 <http://localhost:8080>；匯出到 `outputs/ui/` |

測試：`~/envs/aisc_sim/bin/python -m unittest discover -s tests -v`（G-code 16 個、材料移除 3 個）

規劃中：

| 腳本 | 階段 |
|---|---|
