# 學生操作指南：銑削模擬網頁 UI（M8）

不用看程式碼，照這份跑完一份 G-code、看 3D 動畫、拿到可以送機器人的軌跡檔。

## 啟動

```bash
~/envs/aisc_sim/bin/python scripts/m8_ui.py --port 8080
```

瀏覽器開 <http://localhost:8080>（同一個區網的電腦用 `http://<這台的 IP>:8080`）。第一次開會載入機器人網格，約 10 秒。

## 畫面

左邊是控制面板，右邊是 3D 場景（滑鼠左鍵旋轉、右鍵平移、滾輪縮放）。面板由上到下：

| 區塊 | 做什麼 |
|---|---|
| **1. Cell 與機器人** | 選 `single_robot_table`（一台）或 `dual_robot_table`（兩台）；選要編程的那台；要不要顯示圍籬 |
| **2. G-code** | 從 `assets/gcode/` 選範例，或 **上傳** 自己的 `.nc`。下方會立刻顯示段數、切削長度與 XYZ 範圍；有警告會標 ⚠️ |
| **3. 刀具** | 從刀具庫選，或直接改直徑、鼻端到刀尖長度、刀型（flat／ball）、轉速；按 **儲存為新刀具** 會寫到 `configs/tools/<名稱>.yaml`，之後所有腳本都能用 |
| **4. 工件位置與材料** | 程式原點（G-code 的 X0 Y0 Z0）在機器人底座座標的位置 [mm]、程式 X 軸方位 [deg]、工件尺寸；材料決定切削力估計的比切削能 |
| **5. 取樣與檢查** | G0 速度、取樣間距、Newton 碰撞檢查頻率（30 Hz 約 15 s；60 Hz 較準但慢一倍）、高度圖解析度 |
| **6. 執行** | **執行模擬**：解析 → 取樣 → IK → Newton 碰撞檢查 → 材料移除／切削力／偏移。進度條與狀態會更新；結果列在下方。**匯出軌跡給 ROS 2 bridge** 會寫到 `outputs/ui/<時間>_<程式>_<機器人>/trajectory/` |
| **7. 播放** | 勾 **播放** 看動畫（倍速可調），或拖時間軸 |

## 結果怎麼讀

- **IK ✅／❌**：位置誤差要 < 0.05 mm、刀軸 < 0.05°、未收斂 0 點、速度在上限內。❌ 通常是工件放太遠或太近（改第 4 區的原點），或程式超出工件。
- **Newton 碰撞 ✅／❌**：非預期碰撞要 0。刀具或主軸在切削段進工件是預期的；主軸碰工作台、手臂碰圍籬、快速移動時刀具還在工件裡都算非預期。第一個非預期碰撞的時間與配對會列出來，拖時間軸去看。
- **材料移除**：橘線是切削路徑、灰線是快速移動；米色面是加工後的表面（2.5D 高度圖）。移除體積、最深深度、MRR 可對照程式。
- **切削力／偏移**：一階估計（比切削能 × MRR；關節扭轉彈簧）。材料的 k_c 與關節剛性目前是文獻量級的佔位值，**只能比較趨勢，不能當絕對值**（`docs/SPEC.md` §8）。

## 匯出後要送機器人

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 launch milling_bridge play_trajectory.launch.py trajectory_dir:=$PWD/outputs/ui/<資料夾>/trajectory output_dir:=$PWD/outputs/ui/<資料夾>/session dry_run:=true
```

`dry_run:=true` 只印摘要；確認沒問題再改 `false`，並從 `speed_scale:=0.1` 開始。真機注意事項在 `ros2_ws/README.md`：主軸 TCP 與工件位置沒實測前，只在空中、不裝刀。

## 常見問題

- **上傳後解析失敗**：看 ⚠️ 訊息。不支援刀具補正（G41/G42/G43）、固定循環（G8x）、G92；請在 CAM 後處理時關掉。
- **執行很久**：碰撞檢查頻率降到 20 Hz、取樣間距改 2 mm；長程式（> 10 分鐘）本來就要幾十秒。
- **想要影片或 Isaac Sim 渲染**：匯出軌跡後用 `scripts/m3_sim_replay.py --traj <資料夾>/trajectory` 與 `scripts/m3_render_isaacsim.py`（見 `scripts/README.md`）。
