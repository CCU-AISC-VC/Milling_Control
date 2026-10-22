# DEVLOG

> 只往後加，不修改、不重整舊紀錄。
> 每個任務結束就補一筆。這份是寫給半年後的自己，以及接手的學弟妹看的。

格式：

```
## YYYY-MM-DD ｜ <標題>
- 做了什麼：
- 為什麼這樣做：
- 結果：
- 下一步：
```

---

## 2026-10-22 ｜ 規劃：FANUC 銑削模擬與 sim2real 平台

- **做了什麼**：盤點既有的 `FANUC_Spindle_Modified` ROS 2 工作區（M-710iD/70 ＋ 主軸 URDF、MoveIt 設定、兩個測試腳本，沒有 commit）、實驗室其他 repo 的慣例（`eFlesh-Newton`、`INSTINCT_TwinForge`）、共用環境 `~/envs/aisc_sim` 的版本與 API（Newton 1.7.0.dev0 有 `add_urdf`、`newton.ik`、`ViewerGL/USD/RTX`；cuRobo 0.8.0 模組名稱全改，`curobo.wrap` 不存在；Isaac Sim 6.1 pip）。寫出 M0–M6 路線圖 `plans/2026-10-22_00_roadmap.md`。
- **為什麼這樣做**：老師要一個 sim 與 sim2real 共用的平台，之後要雙機協同加工；repo 結構照實驗室慣例，接手的人不用重新學。
- **結果**：使用者決定：G-code 自己生成測試、第二台也是 M-710iD/70、主軸 TCP 沿用 xacro（標 UNVERIFIED）、ROS 2 工作區搬到 `ros2_ws/`、材料移除留到之後。cuRobo 保留給 G0 避障規劃與對照。
- **下一步**：M0。

## 2026-10-22 ｜ M0：環境與資產

- **做了什麼**：
  - 建 repo 骨架（`docs/`、`plans/`、`scripts/`、`src/milling_sim/`、`configs/`、`assets/`、`outputs/`、`ros2_ws/`）、`AGENTS.md`／`CLAUDE.md`、`instructions.md`、`SPEC.md` r0。既有兩個 ROS 2 套件原封不動搬到 `ros2_ws/src/`；刪掉舊的 `build/install/log`（colcon 可重建）與套件目錄裡混進來的 ament 產物（`environment/`、`hook/`、`*.dsv`、`cmake/`）。
  - `scripts/m0_flatten_urdf.py`：把 `$(find m710id_description_spindle)` 換成原始碼路徑後跑 xacro（不用先 colcon build），`package://` 改成相對路徑，網格連 md5 一起複製到 `assets/fanuc_m710id70_spindle/`。
  - `src/milling_sim/urdf_kinematics.py`：獨立的 NumPy FK 與幾何 Jacobian（解析 Jacobian 與數值 Jacobian 差 1.5e-10）。
  - `src/milling_sim/newton_robot.py`：`add_robot()` 把 URDF 加進 `ModelBuilder` 並記住 body／joint 索引（多台機器人共用一個 builder），`KinematicScene` 做 `eval_fk`。
  - `scripts/m0_fk_check.py`：1000 組隨機關節角對照；`home`／`ready` 快照用 `ViewerGL(headless=True)` 畫 TCP 座標軸。
- **為什麼這樣做**：之後的 IK 與模擬都以這個 URDF 為準，要先證明 Newton 讀進去的和 URDF 寫的一樣；快照是為了確認刀軸方向。
- **結果**：
  - FK 對照通過：`spindle_tcp` 位置差最大 **0.0010 mm**、平均 0.0003 mm；姿態差最大 2.9e-5°（Newton float32 的精度）。`home` TCP 在 (1.490, 0, 1.460) m。
  - **刀軸是 `spindle_tcp` 的 +X**，不是 xacro 註解說的 +Z：主軸 STL 在 `spindle_link` 座標的長軸沿 X（PCA (−0.994, 0, −0.106)），TCP (0.170, 0, 0.120) 在網格 +X 端外 4 mm；快照裡 `home` 姿態主軸垂直朝下、TCP 在刀尖、紅色 +X 沿主軸指向下。寫進 SPEC §3.2 與 `configs/robots/fanuc_m710id70_spindle.yaml` 的 `tool_axis_in_tcp`。
  - Newton 1.7.0.dev0 的 API 與文件不同的地方：`Model.body_key` 不存在，是 `body_label`（值是 `<robot name>/<link>`）；`ViewerGL.log_points/log_lines` 的 `colors` 寫 tuple 會在 `_update_vbo` 炸掉，要給 `wp.array(..., dtype=wp.vec3)`。
  - Newton 匯入的 body 質量：手臂各 link 沒有 `<inertial>`，Newton 從碰撞網格算（base 62.8 kg、J1 103 kg…），只影響動力學，運動學重播不用。
- **下一步**：M1 G-code 解析與取樣。

## 2026-10-22 ｜ M1：G-code 解析與取樣

- **做了什麼**：
  - `src/milling_sim/gcode.py`：SPEC §4 方言的解析器。模態狀態（單位、G90/91、平面、F、S、主軸）；圓弧支援 I/J/K（從起點的增量）與 R（正負號選短／長弧）、全圓、螺旋（平面法向軸線性內插）、P 多圈；G41/42/43、G8x、G92 直接丟 `GCodeError`，其他不認識的 G/M 記 warning。
  - `src/milling_sim/toolpath.py`：取樣（直線依最大間距，圓弧依弦差 `r(1−cos(dθ/2)) ≤ tol` 與間距）；時間律是每段梯形速度曲線、路徑加速度 1 m/s²，兩段切削的夾角 ≤ 5° 就不停、否則停到 0；快速／切削之間一律停。
  - `src/milling_sim/sample_gcode.py` 與 `scripts/m1_make_sample_gcode.py`：自製範例 `assets/gcode/sample_pocket_contour.nc`（R10 圓角輪廓兩層、螺旋下刀 ＋ 同心圓口袋兩層、G2/G3 的 S 槽），與 100 mm 正方形。
  - `scripts/m1_parse_and_sample.py`：解析、取樣、路徑圖、速度曲線、`samples.csv`、`summary.json`。
  - `tests/test_gcode.py`：15 個 unittest（段長、I/J 與 R 圓弧的圓心／掃掠角／方向、螺旋長度、G91、G20、模態、G18 平面、不支援指令、註解與 M30、弦差與間距、時間律、連續性）。
- **為什麼這樣做**：沒有 CAM 輸出可用（使用者決定自己生成），所以範例刻意把 SPEC 列的每種指令都用到；時間律先做簡單可驗證的梯形，之後要接真機的加減速再換。
- **結果**：範例 50 段（8 快速、22 直線、1 順時針弧、19 逆時針弧），切削長 2804.1 mm、快速 463.1 mm；3283 個樣本，最大弦差 0.0124 mm（設定 0.02）、最大間距 1.0 mm、總時間 240.4 s、最大路徑加速度 0.85 m/s²（設定 1.0）、無 warning。unittest 15 個全過。
  - 踩坑：`Segment.point()` 對純量 s 在直線與圓弧的回傳形狀不一致，導致切線計算炸掉；統一成 `np.shape(s) + (3,)`。
  - 取樣邊界的樣本標記為前一段，所以檢查「區間速度 ≤ 進給」時要用後一個樣本的進給。
- **下一步**：M2 IK。

## 2026-10-22 ｜ M2：工件座標與 IK

- **做了什麼**：
  - `src/milling_sim/cell.py`：讀 robot／cell YAML，算 `base ← program`、工件方塊在世界的位置；`robots` 是清單，M6 直接加第二台。
  - `src/milling_sim/ik.py`：5 自由度（位置 ＋ 刀軸）LM IK，null-space 往參考姿態拉，逐點追蹤；起點用「參考姿態 ＋ 手腕網格 ＋ 隨機」種子，在收斂且極限餘裕 ≥ 0.1 的解中選離參考姿態最近的。
  - `src/milling_sim/traj_io.py`：SPEC §5 軌跡檔讀寫。
  - `scripts/m2_ik_trajectory.py`：整條流程、檢查、四張圖、`trajectory/`。
- **為什麼這樣做**：
  - 第一版種子只看「極限餘裕最大」，選到 J5 = −84°、J4 ≈ 99°、J6 ≈ −132° 的扭手腕解（餘裕 0.36，被 J5 卡住）；用結構化種子掃過才發現有 J4 = J6 = 0、J5 = 43° 的直手腕解，餘裕 0.30（被 J3 的非對稱範圍卡住）。兩者餘裕差不多，但直手腕才是銑削要的，所以改成「離參考姿態最近」當準則。
  - null-space 只有 1 維（繞刀軸旋轉），拉不動 J5 這類會改變刀軸的關節，所以解的分支由第一點決定，第一點的種子選擇才是關鍵。
  - 容差從 10 µm 收到 2 µm，平均每點 1.0 → 1.86 次迭代，3283 點 3.1 s，值得。
- **結果**（`outputs/m2_ik/sample_pocket_contour/`，工件原點佔位值 (1.3, −0.1, 0.70) m）：FK(IK) 位置誤差最大 **0.0014 mm**、刀軸 0.0011°、未收斂 0 點、相鄰樣本最大關節差 0.22°、關節速度最大為上限 × 0.5 的 5.4%、極限餘裕最小 0.25（J3）、可操作度最小 0.53；Newton FK 抽 206 點交叉比對 0.0014 mm。關節範圍：J1 −4.4…3.4°、J2 −1.7…9.8°、J3 −44.4…−32.7°、J4 ±4°、J5 42.2…43.5°、J6 ±2.9°。**檢查點 1 通過。**
  - 要記得：這條軌跡依賴 ⚠️ UNVERIFIED 的 TCP 偏移與工件位置，只證明流程可行，不能直接拿去真機切。
- **下一步**：M3 Newton 模擬與渲染。

## 2026-10-22 ｜ M3：Newton 模擬與 Isaac Sim 渲染

- **做了什麼**：
  - `src/milling_sim/sim.py`：由 cell 設定建 Newton 場景（機器人 `add_robot`、工作台與工件為靜態方塊、工件半透明）、`CollisionPipeline`、接觸的有號距離、碰撞分類（`classify_contact`、`in_stock_allowed_mask`）。
  - `scripts/m3_sim_replay.py`：軌跡依 `t` 內插到 60 Hz 運動學重播、TCP 追蹤誤差、各 shape 對最小距離、碰撞判定；`ViewerUSD` 錄 USD（先寫暫存檔再 `os.replace`，避免覆蓋正在被 Isaac Sim 讀的檔）；`ViewerGL` headless 出 overview／closeup 的 MP4 與 GIF（10 倍時間壓縮，刀尖軌跡畫橘線）。
  - `scripts/m3_render_isaacsim.py`：Isaac Sim 6.1 headless 開 USD，加 dome／distant／sphere 光、地板、相機環繞，Replicator `BasicWriter` 出每幀 PNG，ffmpeg 編成 MP4。做法照 `AISC_sim_env/tests/t4_rtx_replay.py`。使用者 2026-10-22 同意 EULA。
- **為什麼這樣做**：運動學重播就夠回答「軌跡能不能走、會不會撞」；切削物理留 M5。碰撞判定要分「預期」（刀在工件裡切削）與「非預期」，否則切削本身就會被算成碰撞。
- **踩坑**：
  - 第一版 cell 設定把工件原點 z 放在工作台面 0.70，而原點定義是工件**頂面**角落，結果整塊工件埋在工作台裡、主軸切削時「穿透工作台」4785 frames。改成 0.75（台面 + 工件高）。
  - 切完一層後的 `G0 Z10` 退刀起點在工件裡，用「motion == rapid」分組會把退刀和後面的橫移 G0 併成一段、方向不是純 +Z 而被判非預期；改成依 G-code 段分組。
  - 接觸面剛好貼合時有號距離是 −1e-7 m 的浮點雜訊，加 0.05 mm 容差。
  - venv 的 imageio 沒有 ffmpeg 後端（使用者指示不再裝任何套件），MP4 改成 pipe 給系統 `ffmpeg`。
  - Replicator 的 `output_dir` 給相對路徑會寫到 `~/omni.replicator_out/` 底下；腳本改成一律 `resolve()`。第一次渲染開頭 4 幀「renderer failed to advance」（shader 編譯），240 幀少 4 幀。
- **結果**（`outputs/m3_sim/sample_pocket_contour/`）：14,427 frames（240.4 s @ 60 Hz）重播 33 s；TCP 追蹤誤差最大 **0.0012 mm**；非預期碰撞 **0** frames；主軸–工件最深 −2.001 mm，預期 −2.001 mm（程式最深 6 mm − 網格刀尖差 4 mm）；其他 shape 對最小距離 48 mm（主軸–工作台）。GL 影片 723 frames × 2 視角；USD 2.1 MB；Isaac Sim RTX 1280×720、16 subframes、236 幀、282 s（首次含 shader 編譯）。**M3 通過。**
- **下一步**：M4 bridge（假硬體驗證）。

## 2026-10-22 ｜ M4：Sim2Real 介面（milling_bridge）

- **做了什麼**：
  - ROS 2 套件 `ros2_ws/src/milling_bridge`（ament_python）：`trajectory_player`（讀 SPEC §5 軌跡資料夾，先慢速走到第一點，再把整條軌跡依 `speed_scale` 時間縮放送 `FollowJointTrajectory`；`dry_run` 預設 true、`speed_scale` 預設 0.1、> 1 要 `allow_fast`）、`joint_state_recorder`（`/joint_states` → CSV，player 事件 → `events.json`）、`mock_hardware.launch.py`（ros2_control 假硬體）、`play_trajectory.launch.py`。
  - `urdf/m710id_70_spindle_ros2_control.urdf.xacro`：既有 MoveIt 設定的 spindle xacro 沒有 `<ros2_control>`，ros2_control_node 會 `std::runtime_error`；bridge 自備 xacro 組合 description 巨集 ＋ MoveIt 的 ros2_control 巨集，不改既有套件。
  - `scripts/m4_run_mock_test.sh`（起假硬體 → 錄製 → 播放 → 收尾）、`scripts/m4_compare_sim_vs_ros.py`（依 `main_sent` 時間戳與 speed_scale 對齊，內插比較，掃 ±0.1 s 找控制器延遲）。
- **為什麼這樣做**：sim2real 的核心是「同一份檔案兩邊都能跑」，所以 ROS 端只讀 `t` 與 `J1`–`J6`，不依賴 `milling_sim`；安全預設照 instructions §三-6。
- **踩坑**：`/opt/ros/jazzy/setup.bash` 和 `set -u` 不相容；`ros2 run`／`ros2 launch` 是 wrapper，對 wrapper 送 SIGINT 不會停掉節點，要用 `setsid` ＋ 對 process group 送訊號或 `pkill -f`；rclpy 在 Jazzy 收到 Ctrl-C 丟 `ExternalShutdownException` 而不是 `KeyboardInterrupt`。
- **結果**（`outputs/m4_bridge/mock_unit_square/`，100 mm 正方形軌跡，speed_scale 2.0 在假硬體上跑 20.5 s）：approach 與 main goal 都 `error_code 0`；錄到 100 Hz 的 joint states；關節曲線與軌跡檔的最大誤差 **0.00069 rad（0.04°）**，已低於 1e-3 rad 門檻；最佳時間偏移 −10 ms（控制器延遲），補償後最大誤差 **0.00010 rad（0.006°）**。**M4（假硬體部分）通過；檢查點 2 的 sim 端成立，真機端待 FANUC 電腦驗證。**
- **M6 預留確認**：`configs/cells/dual_robot_table.yaml` 兩台 M-710iD/70 對向放置可被同一套 `build_cell_scene` 載入（22 bodies、34 shapes、12 coords），兩台之間的碰撞對有回報；兩台都在 `home` 時主軸互相穿透 16.7 mm，M6 的姿態規劃要處理。
- **下一步**：真機（FANUC 電腦）：空中、不裝刀、`speed_scale 0.1` 跑 unit square，比對 real vs sim；確認 UTOOL／UFRAME 後更新 SPEC §3.2；之後 M6。

## 2026-10-22 ｜ M6：雙機協作（共用工件、圍籬、RTX demo）

- **做了什麼**（使用者指示「直接搞雙機協作、demo 漂亮一點、旁邊還有圍籬」）：
  - cell 設定擴充（SPEC r3 §2.2）：每台機器人各自的 `programs` 座標、`stocks` 實體工件、`environment`（地板、圍籬立柱與半透明護板、入口、控制櫃、底座板）、`cameras`。`configs/cells/dual_robot_table.yaml`：兩台 M-710iD/70 對向 3.0 m，共用 600 × 200 × 50 mm 長工件，各切自己那端的 200 × 200 區域、中間留 200 mm；圍籬 6.4 × 4.8 m、高 2 m、−Y 側 1.4 m 入口。
  - 第二份範例 G-code `sample_face_boss.nc`（之字面銑、圓形凸台兩層、兩條槽；62 段、4803 mm、312 s）。
  - `src/milling_sim/replay.py`：多機重播（共用 60 Hz 時間表、較短的那條結束後停住）、碰撞分類、三個視角的 GL 影片、USD（只含機器人＋工作台＋工件，環境由 RTX 腳本重畫）。
  - `src/milling_sim/clearance.py`：機器人間淨空改用 Warp 無號 mesh 點查詢（見下）。
  - `scripts/m6_dual_replay.py`；`scripts/m3_render_isaacsim.py --cell --camera`：用 USD 材質重畫圍籬（護板 opacity 0.35）、地板、控制櫃，相機從 cell 設定讀、環繞。
- **為什麼這樣做**：共用一塊工件最能表現「兩台協同加工」；中間 200 mm 安全帶是依主軸本體幾何（刀軸外側往對向多伸 0.09 m）算的，再用淨空度量證明。
- **踩坑（重要）**：
  - **Newton 的 mesh–mesh 接觸候選不能當淨空**：t = 12.4 s 兩支主軸實際相距 108 mm（頂點對頂點、AABB 間隙 90 mm），Newton 回報 −200 mm「穿透」，`eval_rigid_contact_kinematics` 算出來也一樣，是法向量與點對方向不一致（遠距離的投機接觸）。box–mesh（工作台、工件、圍籬）的有號距離在 M3 驗證過，保留。
  - 第一版用 `mesh_query_point_sign_normal` 取有號距離，主軸 STL 不是 watertight，遠點也被判成「內部」（−254 mm）；改成無號查詢，淨空 < 5 mm 視為接觸（頂點間距 ≤ 10 mm，交錯的表面必有頂點落在 5 mm 內）。
  - 細分到 8 mm 讓主軸網格變成 390 萬頂點、每 frame 650 ms；改成只細分邊長 > 20 mm 的粗網格、查詢點上限 12 萬、每 2 frames 查一次 → 每次 3–40 ms。
  - USD prim 名稱不能有 `-`／`+`，而且 `fence_panel_-y_0` 與 `fence_panel_+y_0` 消毒後會撞名，要加索引。
- **結果**（`outputs/m6_dual/`）：18,731 frames（312 s @ 60 Hz）重播 250 s，62,852 次網格查詢；fanuc_1 TCP 追蹤最大 0.0011 mm、fanuc_2 0.0007 mm；**非預期碰撞 0 frames**；**兩台之間最小淨空 78.5 mm**（兩支主軸，約 t = 14 s，兩台都在靠中間的邊緣）；機器人與圍籬／控制櫃全程沒有接觸候選（> 接觸管線的搜尋範圍）；主軸–工件最深 −2.001／−1.001 mm = 程式最深 6／5 mm − 4 mm。GL 影片三個視角（10 倍速）；Isaac Sim RTX：overview 300 幀環繞 50°、148 s；closeup 與 side 各 300 幀。**M6 通過（佔位擺位下）。**
- **下一步**：擺位、工件、圍籬尺寸全部要以實際 cell 量測取代（SPEC §2.2 ⚠️）；工件掛在第二台 TCP 的模式尚未做；兩台的真機驅動要兩個 `fanuc_ros2_driver` 命名空間，bridge 要加 `--robot` 前綴。

## 2026-10-22 ｜ M7：刀具庫、材料移除、切削力與偏移

- **做了什麼**：
  - `src/milling_sim/tool.py` ＋ `configs/tools/{flat_10mm,flat_6mm,ball_8mm}.yaml`：刀尖 = 主軸鼻端（`spindle_tcp`）+ 長度 × 刀軸。`ik.py` 以刀尖為目標（刀尖 Jacobian = J_v − [r]× J_ω）；cell `robots[].tool`；`sim.py` 在主軸 link 上加鼻端→刀尖的圓柱碰撞體（角色 `<robot>/tool`）；軌跡檔 `x y z` 改為刀尖、`meta.tool`。
  - `src/milling_sim/removal.py`：Warp 高度圖，每對切削樣本掃 2D 膠囊，區間內取最低刀底（修過：純下刀時 L2 = 0 要取 min(z0, z1)，一般段要解二次式取區間端點與最近點）；`tests/test_removal.py`（直槽體積誤差 < 2%、球刀剖面、重切不移除、力的單位）。
  - `src/milling_sim/cutting.py`：k_c × MRR／v_c 的切削力、固定比例的徑向與軸向分量；關節扭轉彈簧的靜態刀尖偏移與關節力矩。`configs/materials.yaml`、robot YAML 的 `joint_stiffness_Nm_rad`（都是量級佔位 ⚠️）。
  - `scripts/m7_material_removal.py`（單機與多機，樣本依全域時間合併）；`scripts/m3_sim_replay.py` 改用 `replay.py`（刀具角色、刀尖追蹤），與 M6 腳本同一條管線。
- **結果**：
  - 單機範例（10 mm 平刀，鋁 6061）：移除 55,812 mm³、最深 6.00 mm（高度圖看得到輪廓槽 4 mm、口袋 6 mm 含螺旋入刀留下的中心小島、S 槽 2 mm）；MRR 最大 451 mm³/s；力最大 83 N（平均 50 N）；刀尖靜態偏移最大 0.39 mm（多在 z）；關節力矩最大 109 N·m。
  - 雙機共用工件：總移除 88,426 mm³（fanuc_1 55,812、fanuc_2 6 mm 平刀 32,614）。
  - 重播改刀具後：單機刀具–工件最深 −6.001 mm = 程式最深（主軸本體離工件 58 mm）、非預期碰撞 0、刀尖追蹤 0.0010 mm；雙機非預期碰撞 0、兩台最小淨空 78.5 mm。
- **注意**：k_c、關節剛性、刀具長度都是佔位值，數字只能比較趨勢。
- **下一步**：M8 UI。

## 2026-10-22 ｜ M8：學生操作 UI（viser）

- **做了什麼**：`src/milling_sim/pipeline.py`（plan → Newton 碰撞檢查 → 材料移除，與腳本同演算法）、`src/milling_sim/ui/app.py`（viser：cell／機器人／G-code 上傳／刀具庫與自訂刀具儲存／工件位置／材料／取樣參數／執行／結果／匯出／播放）、`scripts/m8_ui.py`、`docs/UI_GUIDE.md`。
- **為什麼這樣做**：viser 已在共用環境（Newton 與 cuRobo 的檢視器都用它），不用裝東西；機器人顯示用 URDF visual STL ＋ NumPy FK，Newton 只在執行時做碰撞檢查。
- **結果**：無頭測試：範例 G-code 從解析到材料移除 16.7 s（碰撞檢查 20 Hz，9.9 s）；結果與腳本一致。瀏覽器截圖在 `outputs/ui/`。

## 2026-10-22 ｜ M9：Newton 動力學（關節 PD、重力、切削外力）＋ 影片裡的切削工件

- **做了什麼**：
  - `src/milling_sim/dynamics.py`：`SolverMuJoCo`（implicitfast、關接觸）、關節 PD 位置致動器（`JointTargetMode.POSITION_VELOCITY`，`pd_gains` 在 robot YAML）、重力前饋 τ_g(q) = Σ J_comᵀ (m g)（NumPy 鏈 ＋ Newton 的質量／質心，以 `control.joint_f` 施加）、刀尖外力（`state.body_f`）。URDF `effort=0` 會讓 MuJoCo 編譯失敗（`actfrcrange`），改成 1e5 N·m 佔位上限；`Control.joint_target_pos` 在 Newton 1.5 後改名 `joint_target_q`。
  - `scripts/m9_dynamics.py`：三種模式整條軌跡（模擬 500 Hz、控制 250 Hz、先靜置 1 s），刀尖誤差／關節誤差／力矩圖。
  - 影片與 USD 裡的工件改成會被切的高度圖實體網格（`replay.py` 的 `StockEvolver`、`removal.solid_from_heightmap`；碰撞仍用隱藏的方塊）。
  - RTX 渲染變形工件的踩坑（花最多時間）：Kit 的 Fabric 場景代理**不會**跟著時間軸更新 mesh 的 `points` 時間取樣；把取樣寫回屬性、關掉 Fabric 代理、開檔後用 pxr 新增 prim 或改可見度都沒用（開檔後的編輯不進渲染）；可見度的時間取樣也不理。最後有效的做法：**開檔前**用 pxr 把每個時間取樣做成獨立的靜態 Mesh prim（另存 `scene_render.usd`，原 prim 停用），每個快照用**時間取樣的平移**切換（在自己的區間放在原位、其他時間放到地下 1 km；Kit 只可靠地跟隨 xform 的時間取樣）。還有兩個細節：Replicator 每步會把時間軸往前推半格（實測評估在 tc + 0.5），所以每個整數時間碼放兩個相同值的 key（k 與 k + 0.999）讓值被保持；時間軸要關 looping 並把結束時間延後，否則最後一幀會繞回 t = 0 或被夾在尾端。另一個坑：Newton 的 USD 把 `faceVertexIndices`／`faceVertexCounts`／`normals` 也寫成時間取樣、沒有預設值，`Get()` 不給時間會拿到 None，快照就沒有面。
- **結果**（`outputs/m9_dynamics/sample_pocket_contour/`，10 mm 平刀、鋁、240 s）：
  - 純 PD：起始姿態重力下垂 0.69 mm（z）；全程刀尖誤差最大 1.08 mm、平均 0.91 mm（幾乎都是重力下垂）。
  - PD ＋ 重力前饋：靜置誤差 0；全程最大 0.31 mm（快速移動的加減速段）、平均 0.04 mm。
  - 再加 M7 切削力（最大 83 N）：最大 0.90 mm、平均 0.28 mm，J5／J6 關節誤差最大 80／131 mdeg；關節力矩最大 J2 331 N·m、J3 292 N·m。
  - 三種模式皆無 NaN；每種模式約 5–9 分鐘（與渲染共用 GPU 時）。
- **注意**：質量／慣量由碰撞網格推算、PD 增益與力矩上限為佔位值；只說明「PD 位置控制在重力與切削負載下的誤差量級」，不代表 FANUC 控制器。
