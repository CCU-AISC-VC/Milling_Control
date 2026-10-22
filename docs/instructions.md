# Agent 工作規則

> 本 repo 的常駐規則。任何 AI agent 開始工作前必須完整讀過，並連同 [`SPEC.md`](SPEC.md) 一起讀。
> 結構沿用 `eFlesh-Newton` 與 `INSTINCT_TwinForge` 的 `docs/instructions.md`。

> ⛔ **大前提：模擬程式一律用 `~/envs/aisc_sim/bin/python` 執行（寫絕對路徑）。** ROS 2 端用 Jazzy。細節見 [`../AGENTS.md`](../AGENTS.md)。

---

## 一、專案基本資訊

| 項目 | 內容 |
|---|---|
| 專案名稱 | **Milling_Control**：FANUC M-710iD/70 加主軸的機器人銑削模擬與 sim2real 平台 |
| 單位 | 中正大學 AISC |
| 目標 | 讀入工件的 G-code，轉成機器人關節軌跡；同一份軌跡在 Newton 模擬（Isaac Sim RTX 渲染）與真機上執行；之後擴充到兩台機器人協同加工 |
| 硬體 | FANUC M-710iD/70 ＋ 主軸（第二台也是 M-710iD/70）；真機由 `fanuc_ros2_driver`（ROS 2 Jazzy）驅動，在另一台電腦 |
| 目前進度 | README §3 階段表（**唯一真實來源**）；逐筆紀錄在 `DEVLOG.md` 最後一筆 |

## 二、技術環境

| 元件 | 版本／位置 |
|---|---|
| 模擬環境 | `AISC_sim_env` 固定入口 `~/envs/aisc_sim`（Python 3.12.15）。本 repo 只記錄用的是哪個版本組合，不負責安裝 |
| Newton | 1.7.0.dev0（git main）。機器人用 `ModelBuilder.add_urdf` 載入，關節位置目標驅動（運動學重播）；`newton.eval_fk`、`Model.collide` 做 FK 與碰撞檢查 |
| Isaac Sim | 6.1.0.0（pip）。只負責 RTX 渲染（USD 重播），不負責物理 |
| cuRobo | 0.8.0（模組名稱與 0.7 完全不同：`curobo.motion_planner`、`curobo.inverse_kinematics`、`curobo.robot_parser`；沒有 `curobo.wrap`）。目前未使用，保留給 G0 快速移動的避障規劃 |
| ROS 2 | Jazzy（`/opt/ros/jazzy`）；`ros2_ws/` 內有 `m710id_description_spindle`、`m710id_70_moveit_config` 與本專案的 bridge 套件 |
| 工具 | xacro（ROS 2）、ffmpeg、trimesh、scipy、matplotlib、imageio |

- 不要引入未列在上表的套件；需要時先問。
- 結果要記錄是在哪個環境版本跑出來的（`summary.json` 內寫 Newton 版本）。

## 三、禁止事項

1. **不得修改 `ros2_ws/src/m710id_description_spindle` 與 `m710id_70_moveit_config` 內既有的 URDF 與 MoveIt 設定。** 需要改機器人模型時，改 `assets/` 的攤平版並在 DEVLOG 記錄，之後再回寫。
2. **不得自行產生假的實機資料。** `sessions/` 只放真機錄到的資料；模擬產生的資料一律在 `outputs/`，檔名含 `sim`。
3. **模擬結果與實機結果不得混在同一個數字裡報告。**
4. **SPEC 標 ⚠️ UNVERIFIED 的數值不得當成已確認的事實寫進結論**；尤其是主軸 TCP 偏移與工件在機器人座標的位置。
5. **不確定就問，不要猜。** 尤其是單位、座標系、G-code 方言、工件座標。
6. **真機動作預設 dry-run**，速度比例預設低值；bridge 節點不得在沒有明確旗標時送軌跡給真機。
7. **不得在規劃階段寫程式碼。** 規劃階段只產出 `plans/` 的 PLAN。

## 四、階段交付物（每個階段都要做）

每個階段結束時，在 `outputs/<階段>/` 產出：

1. **圖**：路徑、關節曲線、誤差圖（PNG）；有動作的階段加 GIF 或 MP4（MP4 不進 git，見 `.gitignore`）
2. **數據**：逐步數值 CSV 與 `summary.json`（關鍵數字、設定、環境版本）
3. 補 `docs/DEVLOG.md` 一筆、更新 `docs/PROGRESS.md` 與 README §3、更新 `scripts/README.md`

## 五、工作流程

1. 每個階段先寫 `plans/YYYY-MM-DD_<stage>_<topic>.md`（用 [`../plans/PLAN_template.md`](../plans/PLAN_template.md)）。
2. 路線圖與檢查點：`plans/2026-10-22_00_roadmap.md`。檢查點沒通過就停下來修，不往下一階段推進。
3. 一次只做一個階段，做完回報結果與交付物。
4. 所有回報、文件使用台灣繁體中文；程式碼、檔名、API 名稱維持英文。
