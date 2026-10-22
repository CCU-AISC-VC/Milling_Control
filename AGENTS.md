# AGENTS — 進來先讀

## ⛔ 大前提：模擬程式一律用共用環境跑（必須遵守）

本 repo 的 `scripts/`、`src/`、測試與一次性檢查指令，全部用共用環境的 Python 執行，**寫絕對路徑**：

```bash
~/envs/aisc_sim/bin/python scripts/<script>.py
```

- 共用環境 = [`Kiwi427/AISC_sim_env`](https://github.com/Kiwi427/AISC_sim_env)（固定入口 `~/envs/aisc_sim`，symlink → `~/IsaacLab3/.venv`）。
  本 repo 開工時（2026-10-22）實測版本：Python 3.12.15、Newton 1.7.0.dev0、warp-lang 1.17.0、torch 2.12.0+cu130、
  Isaac Sim 6.1.0.0、cuRobo 0.8.0。環境的安裝與升級照 `AISC_sim_env` README 做，**不在本 repo 處理**，也不 `pip install` 任何東西進去。
- shell 預設沒有啟動任何 venv：直接打 `python`／`python3` 會跑到系統 Python（沒有 Newton）。不要依賴 `source activate` 或 PATH。
- `src/` 不安裝進環境；腳本開頭用 `sys.path` 加入 `src/`（見 `scripts/_bootstrap.py`）。
- **例外只有一個**：ROS 2 端（`ros2_ws/`，要 `rclpy`／MoveIt／`ros2 launch` 的程式）用 ROS 2 Jazzy：
  `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`。真機由另一台電腦上的 `fanuc_ros2_driver` 驅動（見 `ros2_ws/README.md`）。
- 開工前先確認版本：

  ```bash
  ~/envs/aisc_sim/bin/python -c "import sys, newton, warp, torch, isaacsim, curobo; print(sys.version.split()[0], newton.__version__, warp.__version__, torch.__version__, curobo.__version__)"
  ```

- 第一次要啟動 Isaac Sim（Kit）做 RTX 渲染前，NVIDIA Omniverse EULA 要由使用者本人同意；得到同意才設 `OMNI_KIT_ACCEPT_EULA=YES`。

## 接著讀

1. [`docs/instructions.md`](docs/instructions.md) —— 工作規則
2. [`docs/SPEC.md`](docs/SPEC.md) —— 單位、座標系、機器人與 cell 參數、軌跡檔格式的唯一依據
3. [`README.md`](README.md) §3 —— 階段進度表，進度的唯一真實來源
4. [`docs/DEVLOG.md`](docs/DEVLOG.md) 最後幾筆 —— 目前做到哪
5. [`plans/`](plans/) —— 路線圖與每個階段的 PLAN
