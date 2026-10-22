# Milling_Control — FANUC 機器人銑削模擬與 sim2real 平台

**中正大學 AISC 實驗室**。讀入工件的 G-code，轉成 FANUC M-710iD/70 ＋ 主軸的關節軌跡；同一份軌跡在 **NVIDIA Newton** 模擬（**Isaac Sim 6.1 RTX** 渲染）與真機（`fanuc_ros2_driver`，ROS 2 Jazzy）上執行；之後擴充到**兩台 M-710iD/70 協同加工**。

> ⛔ **大前提：模擬程式一律用共用環境 `~/envs/aisc_sim/bin/python` 執行**（寫絕對路徑；系統 `python3` 沒有 Newton）。ROS 2 端用 Jazzy。Agent 進來先讀 [`AGENTS.md`](AGENTS.md)。
>
> 文件分工沿用實驗室其他 repo（`eFlesh-Newton`、`INSTINCT_TwinForge`）：
> [`docs/SPEC.md`](docs/SPEC.md)（單位、座標系、機器人參數、軌跡檔格式的唯一依據）、
> [`docs/instructions.md`](docs/instructions.md)（agent 工作規則）、
> [`docs/DEVLOG.md`](docs/DEVLOG.md)（逐筆開發紀錄）、
> [`docs/PROGRESS.md`](docs/PROGRESS.md)（階段性成果與圖）、
> [`plans/`](plans/)（路線圖與每個階段的 PLAN，先寫再做）。

---

## 1. Overview

```
G-code 檔
  → 解析成工具路徑段（G0/G1/G2/G3、進給、主軸轉速、工件座標）          M1
  → 依弦差與進給率取樣成「時間 + TCP 位姿」序列（program → base_link）   M1
  → 批次 IK 轉成關節軌跡（極限、奇異點、連續性檢查）                    M2
  → 同一份軌跡檔（docs/SPEC.md §5）同時餵給：
       (a) Newton 模擬（碰撞、追蹤誤差）＋ Isaac Sim RTX 渲染           M3
       (b) ROS 2 bridge → fanuc_ros2_driver 的 follow_joint_trajectory   M4
```

設計決定：

- **Newton 只做剛體與運動學重播，不做切削物理。** 機器人用關節位置目標驅動；切削力與材料移除是另一個模型（M5，留到之後）。
- **軌跡檔格式從第一天就定死**（SPEC §5），sim 與真機讀同一個檔。
- **雙機從 M0 就預留**：cell 設定檔是機器人清單；工件座標可以固定，也可以掛在某台機器人的 link 上。
- **IK**：切削路徑逐點用自寫的阻尼最小平方（5 自由度任務：位置＋刀軸，利用繞刀軸旋轉的冗餘避開關節極限）；cuRobo 0.8 負責 G0 快速移動的避障規劃與對照。

## 2. 快速開始

```bash
# 版本確認（要和 AGENTS.md 一致）
~/envs/aisc_sim/bin/python -c "import newton, warp, torch, curobo; print(newton.__version__, warp.__version__, torch.__version__, curobo.__version__)"

# M0：攤平 URDF、FK 對照、快照
~/envs/aisc_sim/bin/python scripts/m0_flatten_urdf.py
~/envs/aisc_sim/bin/python scripts/m0_fk_check.py

# M1–M3：G-code → 軌跡檔 → Newton 重播與影片 → Isaac Sim RTX
~/envs/aisc_sim/bin/python scripts/m1_make_sample_gcode.py
~/envs/aisc_sim/bin/python scripts/m2_ik_trajectory.py
~/envs/aisc_sim/bin/python scripts/m3_sim_replay.py
OMNI_KIT_ACCEPT_EULA=YES ~/envs/aisc_sim/bin/python scripts/m3_render_isaacsim.py   # 需使用者同意 EULA

# M6：雙機 cell（各自 IK → 共用時間軸重播 → RTX）
~/envs/aisc_sim/bin/python scripts/m2_ik_trajectory.py --cell configs/cells/dual_robot_table.yaml --robot fanuc_1 --gcode assets/gcode/sample_pocket_contour.nc --out outputs/m6_dual/traj_fanuc_1
~/envs/aisc_sim/bin/python scripts/m2_ik_trajectory.py --cell configs/cells/dual_robot_table.yaml --robot fanuc_2 --gcode assets/gcode/sample_face_boss.nc --out outputs/m6_dual/traj_fanuc_2
~/envs/aisc_sim/bin/python scripts/m6_dual_replay.py --traj fanuc_1=outputs/m6_dual/traj_fanuc_1/trajectory --traj fanuc_2=outputs/m6_dual/traj_fanuc_2/trajectory
OMNI_KIT_ACCEPT_EULA=YES ~/envs/aisc_sim/bin/python scripts/m3_render_isaacsim.py --usd outputs/m6_dual/scene.usd --cell configs/cells/dual_robot_table.yaml --camera overview --frames 300

# M4：ROS 2 假硬體播放與比對（需先 colcon build ros2_ws）
scripts/m4_run_mock_test.sh outputs/m2_ik/unit_square_100mm/trajectory outputs/m4_bridge/mock_unit_square 2.0
~/envs/aisc_sim/bin/python scripts/m4_compare_sim_vs_ros.py --session outputs/m4_bridge/mock_unit_square --traj outputs/m2_ik/unit_square_100mm/trajectory
```

結果在 `outputs/m0_assets/`。各階段腳本的對照表：[`scripts/README.md`](scripts/README.md)。ROS 2 端（RViz、MoveIt、真機）：[`ros2_ws/README.md`](ros2_ws/README.md)。

## 3. Stages

| ID | Stage | 交付物 | Gate（通過條件，開工前訂） | Status |
|----|-------|--------|------|--------|
| **M0** | 環境與資產：repo 骨架、xacro → URDF、Newton 載入、FK 對照 | `outputs/m0_assets/` | 1000 組隨機關節角，Newton 與 NumPy FK 的 TCP 位置差 < 0.01 mm | ✅ 2026-10-22：最大 0.0010 mm、姿態 2.9e-5° |
| **M1** | G-code 解析與取樣：SPEC §4 方言、進給時間律、自製範例與單元測試 | `outputs/m1_gcode/` | 取樣路徑與理論路徑相差 < 弦差設定；單元測試全過 | ✅ 2026-10-22：最大弦差 0.012 mm（設定 0.02）、15 tests pass |
| **M2** | 工件座標與 IK：逐點 IK、冗餘利用、速度檢查；cuRobo G0 規劃（未做，見 DEVLOG） | `outputs/m2_ik/`、軌跡檔 | FK(IK(x)) < 0.05 mm、刀軸 < 0.05°；不跳變；速度在上限內 | ✅ 2026-10-22：0.0014 mm、0.0011°、0 未收斂、速度 5% |
| 檢查點 1 | 一份自製 G-code 走完 M1→M2 成為可執行的關節軌跡 | | | ✅ 2026-10-22 |
| **M3** | Newton 模擬與渲染：cell、運動學重播、碰撞檢查、USD → Isaac Sim RTX 影片 | `outputs/m3_sim/` | 無非預期碰撞；模擬 TCP 與目標路徑相差 < 0.1 mm | ✅ 2026-10-22：0.0012 mm、非預期碰撞 0、RTX 影片 |
| **M4** | Sim2Real 介面：`milling_bridge` ROS 2 套件、假控制器驗證、joint state 錄製 | `outputs/m4_bridge/` | 同一份檔案在 Newton 與 ros2_control 假硬體的關節曲線一致（< 1e-3 rad） | ✅ 2026-10-22 假硬體：最大誤差 0.00069 rad（補償 10 ms 延遲後 0.00010 rad）；🚧 真機待 FANUC 電腦 |
| 檢查點 2 | sim2real ready：同一份軌跡檔兩邊都能跑 | | | 🚧 sim 端與假硬體 ✅；真機待驗證 |
| M5 | 材料移除（選用） | | 使用者決定 | 留到之後 |
| **M6** | 雙機協作：兩台 M-710iD/70 共用工件各切一端、共用時間軸、機器人間淨空、圍籬與環境、RTX demo | `outputs/m6_dual/` | 各台 TCP 追蹤 < 0.1 mm、無互撞（淨空 > 5 mm）、無非預期碰撞 | ✅ 2026-10-22：追蹤 0.0011 mm、最小淨空 78.5 mm、碰撞 0；工件掛在另一台 TCP 的模式未做 |

Legend: ✅ done · 🚧 in progress · ❌ not started. **這張表是進度的唯一真實來源，每次階段性完成就更新它。**

## 4. 目錄

| 路徑 | 內容 |
|---|---|
| `src/milling_sim/` | 共用程式：`gcode.py`、`toolpath.py`（M1）；`cell.py`、`ik.py`、`traj_io.py`（M2）；`sim.py`、`replay.py`（M3／M6 重播與碰撞分類）；`clearance.py`（M6 機器人間淨空）；`urdf_kinematics.py`、`newton_robot.py`、`viz.py`（M0） |
| `scripts/` | 一個階段一個腳本；對照表在 `scripts/README.md` |
| `configs/robots/`、`configs/cells/` | 機器人設定（刀軸、速度比例）、cell 設定（機器人清單、各台程式座標、工件、工作台、圍籬等環境、相機；單機與雙機各一份） |
| `assets/fanuc_m710id70_spindle/` | 攤平的 URDF 與網格（由 `scripts/m0_flatten_urdf.py` 產生）；`assets/gcode/` 範例 G-code |
| `ros2_ws/src/` | 實驗室既有的 `m710id_description_spindle`、`m710id_70_moveit_config`；本專案的 `milling_bridge`（軌跡檔 → `FollowJointTrajectory`、joint state 錄製、假硬體 launch） |
| `outputs/<階段>/` | 圖、GIF、CSV、`summary.json`（MP4 與 USD 不進 git） |
| `sessions/` | 真機錄製（之後） |
| `docs/`、`plans/` | 見上方 |

## 5. 硬體與環境

| 項目 | 內容 |
|---|---|
| 機器人 | FANUC M-710iD/70 ＋ 主軸（實驗室既有 URDF）；第二台也是 M-710iD/70 |
| 真機 | `fanuc_ros2_driver`（ROS 2 Jazzy），在另一台電腦，robot IP 見 `ros2_ws/README.md` |
| 模擬 | `AISC_sim_env` 共用環境 `~/envs/aisc_sim`：Newton 1.7.0.dev0、Isaac Sim 6.1.0.0、cuRobo 0.8.0、Python 3.12.15 |
| GPU | RTX 5090 Laptop 24 GB（開發機） |

## 6. 已知風險

1. **主軸 TCP 偏移與刀軸方向沒有實測來源**（SPEC §3.2 ⚠️）。確認前真機只在空中動作。
2. **Newton API 每月變動**，`add_urdf`／viewer 的簽章已和文件不同；環境版本固定，結果記錄版本。
3. **真機追蹤誤差未知**：運動學重播假設控制器完美跟隨；M4 之後用實機 joint states 量化。
4. **G0 速度**：G-code 沒定義快速移動速度，真機由 bridge 的速度比例限制。
