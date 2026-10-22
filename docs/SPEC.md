# SPEC — 單位、座標系、機器人參數與資料格式契約

> **Authority.** 本檔是單位、座標系、機器人與 cell 參數、G-code 解讀規則、軌跡檔格式的唯一依據。程式與 agent 必須遵守。
> 若現實與 SPEC 不符，**先停下來修 SPEC，再改程式**，不可默默換慣例。
>
> 標記 ⚠️ **UNVERIFIED** 的項目是佔位符或由間接證據推斷的值，必須對照一手來源（FANUC 控制器的 UTOOL／UFRAME、實際量測、CAM 後處理器設定）確認後，才能讓任何要報告的結果依賴它。
>
> **Revision: r6（2026-10-22，五軸 G-code）**
> r5 → r6：§4 五軸刀軸向量（G0/G1 的 I J K）、取樣依刀軸轉角細分、球刀傾斜的材料移除。見 DEVLOG 2026-10-22 五軸與校徽。
> r4 → r5：§9 動力學模型；§6 影片與 USD 的工件改為隨時間被切的高度圖網格。見 DEVLOG 2026-10-22 M9。
> r3 → r4：§3.5 刀具庫與刀尖定義；§2.2 `robots[].tool`；§8 材料移除、切削力與靜態偏移模型；軌跡檔 `x y z` 改為刀尖位置、`meta.tool`。見 DEVLOG 2026-10-22 M7。
> r2 → r3：§2.2 cell 設定檔的多程式座標、多工件、環境、相機；§6.2 雙機規則與機器人間淨空的量法。見 DEVLOG 2026-10-22 M6。
> r1 → r2：§3.2 加主軸網格刀尖與 TCP 的 4 mm 差；§6 加碰撞判定規則；工件原點 z 改為工作台面 + 工件高（0.75）。見 DEVLOG 2026-10-22 M3。
> r0 → r1：§3.1 加參考姿態；§3.4 IK 參數；§4 加取樣與時間律參數；§5 軌跡檔格式落實到 `traj_io.py`。見 DEVLOG 2026-10-22 M1、M2。

---

## 1. Units

| 量 | 程式內部 | 報告與圖表 | 備註 |
|---|---|---|---|
| 長度 | **m** | mm | G-code 的 mm／inch 在解析器邊界轉成 m（§4） |
| 角度 | **rad** | deg | 只在 `_deg` 後綴的設定鍵與報告中用度 |
| 時間 | **s** | s | 軌跡檔的 `t` 從 0 起算 |
| 進給 | m/s | mm/min | G-code 的 F 在解析器邊界轉換 |
| 主軸轉速 | rpm | rpm | 只傳遞，不模擬 |
| 力／力矩 | N／N·m | N／N·m | 目前沒有力的模擬（§7） |

## 2. Coordinate frames

| Frame | 定義 |
|---|---|
| `world` | Newton 場景根，Z 朝上，右手系。單機 cell 中 `world` = `fanuc_1/base_link` |
| `<robot>/base_link` | 機器人底座，URDF 的根 link。cell 設定檔（`configs/cells/*.yaml`）給每台機器人的 `base_xyz_m`／`base_rpy_rad` |
| `<robot>/flange` | J6 法蘭面，沿 J6 軸向外 0.175 m（URDF `J6-flange`） |
| `<robot>/spindle_tcp` | 工具中心點。**切削路徑的目標 frame。** 偏移見 §3.2 |
| `program` | G-code 的工件座標系（G54）。原點與方位由 cell 設定檔 `workpiece.origin_*` 給，在機器人 `base_link` 下表示。程式的 X／Y／Z 軸與 `base_link` 的 X／Y／Z 對齊（旋轉為 0）時最簡單；其他方位也允許，但要寫進設定檔 |
| `table` | 工作台，靜態方塊；只用於碰撞檢查與渲染 |

### 2.2 cell 設定檔（`configs/cells/*.yaml`）

| 鍵 | 意義 |
|---|---|
| `robots[]` | 每台的 `name`、`robot`（robot YAML）、`base_xyz_m`／`base_rpy_rad`（world ← base）、`tool`（刀具 YAML，§3.5；省略 = 直接以主軸鼻端編程） |
| `programs.<robot>` | 該台的 G-code `program` frame（`frame` 為 `world` 或 `<robot>_base`，`origin_xyz_m`、`origin_rpy_rad`）。沒有時退回單一的 `workpiece` |
| `stocks[]` | 實體工件方塊：原點是工件**頂面**角落，方塊沿 +X、+Y、−Z 延伸（`size_xyz_mm`） |
| `table` | 工作台方塊（`center_xy_m`、`size_xyz_m`、`top_z_m`） |
| `environment` | `floor`（頂面高度、尺寸）、`fence`（矩形範圍、高度、立柱間距、護板厚度與離地間隙、`gates[]` 入口、顏色與護板透明度）、`boxes[]`（控制櫃、底座板等） |
| `cameras.<name>` | `pos`、`target`（GL 影片與 RTX 渲染共用） |

環境方塊全部進 Newton 碰撞檢查（角色 `env/<name>`）；RTX 渲染時由 `scripts/m3_render_isaacsim.py --cell` 用 USD 材質重畫（Newton 的 USD 不帶透明度）。

### 2.1 會咬人的轉換

| 項目 | 規定 |
|---|---|
| URDF 的 rpy | fixed-axis XYZ（`R = Rz(yaw) · Ry(pitch) · Rx(roll)`），`urdf_kinematics.py` 用 SciPy `from_euler("xyz")` |
| 四元數順序 | 程式內部一律 **`(x, y, z, w)`**（Newton、SciPy、ROS 同）；cuRobo 的 `Pose` 與 USD 用 `(w, x, y, z)`，**只在單一轉換函式裡重排**，呼叫端不得手動重排 |
| Newton `body_q` | `(px, py, pz, qx, qy, qz, qw)` |
| 刀軸方向 | 由 cell／robot 設定檔的 `tool_axis_in_tcp` 給，**不在程式裡寫死**；見 §3.2 |
| 3 軸銑削的刀軸 | 程式座標的 **−Z**（刀尖朝下），即 `tool_axis_in_tcp` 要對齊 `program` 的 −Z |

## 3. 機器人：FANUC M-710iD/70 ＋ 主軸

來源：`ros2_ws/src/m710id_description_spindle/urdf/m710id_70.urdf.xacro`（實驗室既有），攤平版在 `assets/fanuc_m710id70_spindle/m710id_70_spindle.urdf`（由 `scripts/m0_flatten_urdf.py` 產生，`manifest.json` 有網格 md5）。

### 3.1 運動學

| 項目 | 值 | 來源 |
|---|---|---|
| 連桿 | base 0.565、J1→J2 x 0.150、J2→J3 z 0.895、J3→J4 z 0.170、J4→J5 x 1.045、J6→flange x 0.175（m） | xacro |
| 關節軸 | J1 +Z、J2 +Y、J3 −Y、J4 −X、J5 −Y、J6 −X | xacro |
| 關節範圍（deg） | J1 ±185、J2 −75…150、J3 −80.5…205、J4 ±400、J5 ±125、J6 ±400 | xacro |
| 關節速度上限（deg/s） | 180、180、180、260、260、370 | xacro（與 FANUC 型錄一致） |
| 關節加速度上限 | URDF 沒有；`configs/robots/*.yaml` 的 `acceleration_limits_rad_s2` 是佔位符 ⚠️ UNVERIFIED | — |
| 銑削時速度比例 | URDF 上限的 0.5（`velocity_scale`） | 本專案安全餘裕，可調 |
| 家姿態 `home` | 全 0；TCP 在 base 座標 (1.490, 0, 1.460) m | M0 FK |
| `ready` | (0, −0.785, 0.785, 0, −1.5708, 0) rad | SRDF |
| 參考姿態 `preferred_state` | `home`（手腕打直，J4 = J6 = 0）。IK 的起點在收斂且極限餘裕 ≥ 0.1 的解中選離它最近的；繞刀軸的自由旋轉也往它拉 | M2，`configs/robots/*.yaml` |

M0 驗證（2026-10-22）：Newton `add_urdf` 與獨立 NumPy FK，1000 組隨機關節角，`spindle_tcp` 位置差最大 0.0010 mm、姿態差最大 2.9e-5°（float32 精度），通過。

### 3.2 主軸與 TCP ⚠️ UNVERIFIED

| 項目 | 值 | 來源與疑點 |
|---|---|---|
| 主軸安裝 | `flange` → `spindle_link`：xyz 0、rpy (0, π/2, 0) | xacro；註解寫「try rpy=0 π/2 0」，是試出來的值 |
| TCP 偏移 | `spindle_link` → `spindle_tcp`：xyz (0.170, 0, 0.120) m、rpy 0 | xacro；註解寫「tune these」，**沒有量測來源** |
| 主軸質量與慣量 | 2.10 kg、Ixx 0.002、Iyy 0.002、Izz 0.0015 | xacro 註解寫「tune these」 |
| **刀軸方向** | **`spindle_tcp` 的 +X**（從主軸本體指向刀尖） | 由網格推斷：主軸 STL 在 `spindle_link` 座標的長軸沿 X（PCA 主軸 (−0.994, 0, −0.106)），x 範圍 −0.279…0.166 m，TCP 在 x = 0.170，剛好在網格 +X 端外 4 mm；在 `home` 姿態 +X 指向世界 −Z（刀尖朝下）。xacro 註解說刀軸是 +Z，**與網格不符**，以網格為準 |

| 網格刀尖 vs TCP | 主軸 STL 在刀軸方向的末端在 x = 0.166 m，TCP 在 0.170 m：**網格比 TCP 短 4 mm**（`mesh_tip_offset_mm: 4.0`）。碰撞檢查預期的最深穿透 = 程式深度 − 4 mm | M0 網格量測、M3 碰撞探針（刀尖放在工件頂面 → 距離 +4 mm；插入 10 mm → −6 mm） |

要確認的做法：在 FANUC 控制器讀 UTOOL 的工具座標（x, y, z, w, p, r），與 `flange → spindle_tcp` 的合成變換比對；或用千分錶／雷射量刀尖。確認前，所有實機動作只在空中、不接觸工件。

### 3.4 IK（M2，`src/milling_sim/ik.py`）

| 項目 | 值 |
|---|---|
| 任務 | 5 自由度：刀尖位置（3）＋ 刀軸方向（2）；繞刀軸旋轉自由 |
| 解法 | Levenberg–Marquardt（λ = 1e-3），位置列權重 1e3（1 mm → 1）、方向列權重 57.3（1° → 1）；null-space 往參考姿態拉（增益 0.2／次）；每次最大步 0.25 rad；硬夾在極限內 2% |
| 收斂容差 | 位置 2 µm、方向 2e-5 rad |
| 逐點追蹤 | 每點以前一點的解當起點；第一點用參考姿態 ＋ 手腕網格（J4／J6 ∈ {0, ±90°, 180°}，J5 ∈ {−60°, 0, 60°}）＋ 32 個隨機種子 |
| 通過門檻 | FK(IK) 位置 < 0.05 mm、刀軸 < 0.05°、未收斂 0 點、相鄰樣本關節差 < 5°、關節速度 ≤ 上限 × `velocity_scale` |

### 3.5 刀具（M7，`configs/tools/*.yaml`、`src/milling_sim/tool.py`）

| 項目 | 規定 |
|---|---|
| 參考 | `spindle_tcp` 當**主軸鼻端**；刀尖 = 鼻端 + `length_from_nose_mm` × 刀軸（+X） |
| 欄位 | `name`、`type`（flat／ball）、`diameter_mm`、`flute_length_mm`、`length_from_nose_mm`、`shank_diameter_mm`、`flutes`、`max_rpm` |
| IK | 目標是刀尖（`ToolAxisIK(tool_length=…)`，刀尖 Jacobian = J_v − [r]× J_ω） |
| 碰撞與視覺 | Newton 在主軸 link 上加一個鼻端→刀尖的圓柱（角色 `<robot>/tool`），切削段與純退刀時刀具或主軸在工件內都算預期；預期最深 = 程式最深（不再減 4 mm 網格差） |
| 軌跡檔 | `x y z` 是**刀尖**在 base 的位置；`meta.tool` 記錄刀具；沒有刀具時等於鼻端（與 r3 相容） |
| 既有刀具 | `flat_10mm`（60 mm）、`flat_6mm`（50 mm）、`ball_8mm`（55 mm）：長度皆為佔位值 ⚠️ UNVERIFIED |

### 3.3 碰撞模型

URDF 的 collision 網格：手臂各 link 的簡化 STL、主軸用與 visual 相同的 73,334 面 STL。Newton 匯入後 16 個 shape（每個 link visual + collision 各一）。自碰撞檢查預設關閉（M3 再決定）。

## 4. G-code 方言（M1）

| 支援 | 規定 |
|---|---|
| 單位 | G21 mm（預設）、G20 inch |
| 座標模式 | G90 絕對（預設）、G91 增量 |
| 移動 | G0 快速、G1 直線、G2／G3 圓弧（I J K 圓心偏移或 R 半徑）；平面 G17（預設）／G18／G19 |
| 工件座標 | G54–G59：全部視為同一個 `program` frame（cell 設定檔只給一個原點）；G92 不支援 |
| 進給 | F（mm/min 或 inch/min，依單位）；模態 |
| 主軸 | S 轉速、M3／M4 開、M5 關；只記錄，不模擬 |
| 程式控制 | M0／M1 暫停視為無動作、M2／M30 結束；N 行號、`( )` 與 `;` 註解忽略 |
| **五軸** | G0／G1 行上的 I J K 是**刀軸單位向量（刀尖 → 主軸）**，模態；G2／G3 行上的 I J K 仍是圓心偏移。只換向不移動的行是「原地重新定向」，以 30°/s 給時間。程式內部 `tool_dir` = 主軸 → 刀尖 = −(I, J, K)。旋轉軸字元 A／B／C 不支援（機台相關） |
| 不支援 | 刀具補正 G41／G42／G43、固定循環 G8x、變數與巨集、G92、旋轉軸字元 A／B／C |
| 取樣 | 直線依最大取樣間距（預設 1 mm）切分；圓弧依弦差（預設 0.02 mm）與最大間距切分，每段端點保留；五軸再依刀軸轉角 ≤ 2°／樣本細分，刀軸在段內 slerp |
| 時間律 | 每段梯形速度曲線，路徑加速度預設 1 m/s²；兩段切削的夾角 ≤ 5° 時以進給連續通過，否則停到 0；快速 ↔ 切削之間一律停到 0 |
| 快速移動速度 | G0 用設定檔的 `rapid_feed_mm_min`（預設 3000）⚠️ 真機的 G0 速度由 bridge 的速度比例限制 |

## 5. 軌跡檔格式（M2 起；sim 與真機讀同一份）

一個軌跡 = 一個資料夾 `<name>/`，內含：

- `trajectory.csv`：每列一個時間點。欄位：`t`、`J1`…`J6`（rad）、`x`、`y`、`z`（TCP 在 `base_link`，m）、`qx qy qz qw`（TCP 姿態）、`feed_mm_min`、`spindle_rpm`、`segment`（G-code 行號）、`motion`（`rapid`／`linear`／`arc`）
- `meta.json`：來源 G-code 檔與 md5、cell 設定檔、robot 設定檔、取樣參數、環境版本、產生時間、`checks`（極限、連續性、可達性的結果）

真機端 `milling_bridge` 只讀 `t` 與 `J1`…`J6`；其他欄位給模擬與分析用。

## 6. 模擬設定（M3）

| 項目 | 值 |
|---|---|
| 機器人驅動 | 運動學重播：每個模擬 frame 直接設關節座標並 `eval_fk`；不模擬馬達動力學 |
| 時間 | 模擬 frame 率 60 Hz；軌跡依 `t` 線性內插到 frame |
| 碰撞檢查 | `newton.CollisionPipeline`（`include_static_kinematic_pairs=True`）：每 frame 從接觸點算有號距離 `dot(p1 − p0, n)`（負 = 穿透），依 shape 對取最小值 |
| 碰撞判定 | **預期**：刀具或主軸–工件，且該時刻刀具允許在工件內（切削段、或緊接在切削後沿刀軸的純退刀 G0）。**非預期**：其他任何穿透超過 0.05 mm（例如 J6 或法蘭碰工件、主軸碰工作台、橫移 G0 時刀具在工件內）。預期最深穿透 = 程式最深（有刀具時）或程式最深 − 網格刀尖差 4 mm（無刀具），容差 1 mm |
| 刀具對方塊的距離 | **不用** Newton 的圓柱–方塊接觸（10 mm 平刀剛好對，2 mm／8 mm 球刀給 −5…−34 mm 的假穿透）。改用解析幾何 `tool_box_distance`：刀具最低點（球刀 = 刀尖；平刀 = 刀尖再減 r·sin(傾角)）相對方塊頂面的垂直距離，腳印在方塊外時給側向間隙 |
| 工件位置 | cell 設定的 `workpiece.origin_xyz_m` 是工件**頂面**角落；z = 工作台面 + 工件高（佔位值 0.70 + 0.05 = 0.75）⚠️ UNVERIFIED |
| 渲染 | 開發：Newton `ViewerGL` headless；交付：`ViewerUSD` 錄 USD → Isaac Sim 6.1 RTX |
| 切削中的工件 | 影片與 USD 裡工件是 `StockEvolver` 的高度圖實體網格（頂面 ＋ 側壁 ＋ 底面，1 mm 格），依影片時間逐步被切；碰撞仍用隱藏的工件方塊。USD 每 5 個影格存一次網格時間取樣（`/root/stock_machined`） |

## 6.2 雙機（M6）

| 項目 | 規定 |
|---|---|
| 時間軸 | 所有機器人共用同一個 60 Hz frame 表；各自的軌跡依 `t` 內插，較短的那條結束後停在最後一點 |
| 機器人–機器人淨空 | **不用** Newton 的 mesh–mesh 接觸候選（兩支主軸相距 108 mm 時曾回報「穿透 200 mm」，法向量與點對不一致）。改用 Warp 的無號 mesh 點查詢：手腕四個 link（J4、J5、J6、主軸）的碰撞網格頂點對另一台網格表面的最短距離，兩個方向取最小；粗網格先細分到 ≤ 10 mm，查詢點上限 12 萬；淨空 **< 5 mm 視為接觸**（`clearance.py`） |
| 機器人–環境、機器人–工作台、主軸–工件 | 仍用 Newton 接觸管線的有號距離（box–mesh 已驗證），規則同 §6 |
| 通過條件 | 各台 TCP 追蹤 < 0.1 mm；非預期碰撞 0 frames；機器人間最小淨空 > 5 mm；主軸–工件最深 = 程式最深 − 4 mm（容差 1 mm） |
| 安全帶 | 共用工件時兩台程式區域之間留 ≥ 200 mm：主軸本體在刀軸外側往對向機器人多伸約 0.09 m，加上手腕寬度 |

## 6.1 真機介面（M4）

| 項目 | 值 |
|---|---|
| 介面 | `FollowJointTrajectory` action `/arm_controller/follow_joint_trajectory`（真機：`fanuc_ros2_driver`；測試：ros2_control `mock_components/GenericSystem`） |
| 時間縮放 | bridge 把軌跡時間除以 `speed_scale`（預設 0.1）；`speed_scale` > 1 只限模擬（`allow_fast`） |
| 對齊 | recorder 以 player 的 `main_sent` 時間戳為 0，軌跡時間 = (t − stamp − lead_in) × speed_scale |
| 假硬體延遲 | 約 10 ms（M4 實測，100 Hz 更新）；比對時掃 ±0.1 s 取最小誤差，原始與補償後都要報告 |

## 8. 材料移除、切削力與靜態偏移（M7）

| 項目 | 模型 |
|---|---|
| 材料移除 | 2.5D 高度圖（world XY 格，預設 0.5 mm；`removal.py`，Warp kernel）。每對相鄰切削樣本掃一個半徑 r 的 2D 膠囊，格子高度降到刀底高度（flat：刀尖 z；ball：球心 z − √(r² − d²)，球心 = 刀尖 − r × 刀軸，所以**傾斜的球刀**也正確），取區間內最低點；移除體積用 atomic 累加 → MRR。平刀傾斜時仍當水平圓盤（2.5D 限制）。**沒有**側壁 undercut、切屑、刀具磨耗；高度圖不知道刃長。驗證：直槽解析體積誤差 < 2%、球刀下刀剖面 |
| 切削力 | 比切削能 k_c（`configs/materials.yaml`，教科書量級 ⚠️ UNVERIFIED）：P = k_c · MRR；v_c = π D n／60；F_t = P／v_c；F_r = 0.4 F_t、F_a = 0.25 F_t（典型比例）。方向：切向沿 −進給、徑向 = z × 進給、軸向 +z（推離工件）。只在有 MRR 的樣本有力 |
| 靜態偏移 | 關節扭轉彈簧 K = diag(k_j)（`joint_stiffness_Nm_rad`，量級佔位 ⚠️ UNVERIFIED）：C = J K⁻¹ Jᵀ（刀尖 Jacobian），δ = C[:3,:] · [F; 0]；關節力矩 τ = Jᵀ [F; 0]。不含重力、連桿柔性、動態 |
| 有效範圍 | 只能比較趨勢（哪段路徑負載大、哪個姿態偏移大）；絕對值要等 k_c 與 k_j 量測後才可信 |

## 9. 動力學（M9，`src/milling_sim/dynamics.py`）

| 項目 | 模型 |
|---|---|
| 求解器 | Newton `SolverMuJoCo`（MuJoCo-Warp），integrator implicitfast，模擬 500 Hz、控制 250 Hz，接觸關閉 |
| 質量／慣量 | URDF 匯入：手臂 link 無 `<inertial>` → Newton 由碰撞網格推算（總約 320 kg；型錄整機約 560 kg）⚠️ UNVERIFIED；主軸 2.1 kg（xacro） |
| 致動器 | 關節 PD 位置致動器（`JointTargetMode.POSITION_VELOCITY`）：τ = kp (q_t − q) + kd (q̇_t − q̇)；`pd_gains` 見 robot YAML（佔位）；力矩上限 1e5 N·m（佔位） |
| 重力前饋 | τ_g(q) = Σ_links J_com(q)ᵀ (m g)（NumPy 鏈 ＋ Newton 的 m、COM）；`control.joint_f = −τ_g`；靜止驗證誤差 0 |
| 切削外力 | M7 的力向量（world）施加於主軸 link 的刀尖：`state.body_f` = (F, (p_tip − p_com) × F) |
| 輸出 | 刀尖誤差（相對 M2 規劃軌跡）、z 向下垂、關節誤差、致動器力矩 |
| 有效範圍 | 說明「PD 位置控制在重力與切削負載下的追蹤誤差量級」；不代表 FANUC 控制器（有前饋與積分）、不含減速機柔性與摩擦 |

## 7. 模型有效範圍

本平台模擬的是**運動學與幾何**：關節軌跡、刀尖路徑、碰撞、可達性、時間；M7 之後加上 2.5D 材料移除與一階的切削力／靜態偏移估計（§8）。**不模擬**真實切削物理、主軸動態、手臂動力學與控制器追蹤誤差。sim 與真機的差異要由 M4 的關節曲線比對量化，不能假設為零。
