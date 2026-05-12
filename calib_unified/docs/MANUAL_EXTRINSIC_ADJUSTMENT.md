# 外参手动调整功能说明

## 1. 目的

当自动标定精度不足（如 RMS 超过阈值）或需要人工微调时，可通过 **6-DOF 键盘交互** 微调外参。支持的外参类型：

| 外参类型 | 说明 |
|----------|------|
| **LiDAR-Camera** | 激光雷达与相机外参，点云投影到图像上观察边缘对齐 |
| **Camera-Camera** | 相机与相机外参，对极线/对应点可视化 |
| **LiDAR-LiDAR** | 激光雷达与激光雷达外参 |
| **IMU-LiDAR** | IMU 与雷达外参（旋转可视化验证） |

## 2. 如何启用手动标定

### 2.1 通过 calib_unified_run.sh（推荐）

在 `--run --task <任务>` 后加上 **`--manual`**，**只要使用了 `--manual`，自动标定完成后就会自动打开可视化界面**，进入手动校准阶段并弹出 6-DOF 调整窗口（与 RMS 是否超阈值无关）。

```bash
# LiDAR-相机外参 + 手动调整
./calib_unified_run.sh --run --task lidar-cam --manual

# 含粗标定
./calib_unified_run.sh --run --task lidar-cam --coarse --manual

# 相机-相机、雷达-雷达、IMU-雷达
./calib_unified_run.sh --run --task cam-cam --manual
./calib_unified_run.sh --run --task lidar-lidar --manual
./calib_unified_run.sh --run --task imu-lidar --manual
```

运行上述命令后，**终端会打印「手动标定操作说明」**（按键与步骤），注意查看。

### 2.2 独立可执行文件

```bash
# 容器内或本地已编译时
unicalib_lidar_camera --config config.yaml --manual
# LiDAR-Camera 使用 Pangolin 点云叠加+左侧 6-DOF 面板（方案 B，需编译时启用 Pangolin）
unicalib_lidar_camera --config config.yaml --manual --pangolin-panel

unicalib_cam_cam      --config config.yaml --manual
unicalib_lidar_lidar  --config config.yaml --manual
unicalib_imu_lidar    --config config.yaml --manual
```

### 2.3 配置与阈值

- 配置中可设置 `allow_manual_fallback: true`，或由应用层传 `--manual`。
- 质量阈值（超过则建议/进入手动）：
  - LiDAR-Cam: `lidar_cam_rms_threshold` / `manual_rms_threshold`（默认约 2.0 px）
  - Cam-Cam: `cam_cam_rms_threshold` / `manual_rms_threshold`（默认约 1.5 px）

## 3. 详细操作步骤

1. **启动**：执行上述带 `--manual` 的命令（或配置中启用手动回退）。
2. **等待精标定**：程序先完成粗标定（若启用）、精标定，并缓存一帧数据供手动阶段使用。
3. **弹出窗口**：精标定结束后，若启用了手动，会弹出可视化窗口：
   - **LiDAR-Camera**：主视图为「LiDAR 点云投影到相机图像」的叠加图，调整 6-DOF 后实时重绘，可直接观察边缘/特征对齐效果；同时会弹出 **3D 点云窗口**（LiDAR 系下点云 + 相机坐标系），便于查看周围环境、辅助手动调整。
   - **Camera-Camera**：主视图为**左右目图像水平拼接 + 对极线**，调整 6-DOF 后对极线实时更新；对极线对齐越好表示标定越准。
   - **LiDAR-LiDAR**：主视图为**双 LiDAR 点云融合**（Ref 固定 + Target 按当前外参变换到 Ref 系），左侧 Pangolin 面板为 6-DOF ± 按钮、Intensity Color、Point Size、Reset、Accept；按键 q/a,w/s,e/d,r/f,t/g,y/h 与 U/Enter/Esc 同上，两片点云对齐越好表示外参越准。
   - 窗口上会叠加 T (m) 与 RPY (deg) 及按键说明。
4. **按键微调**：按下面「按键说明」调整旋转与平移。LiDAR-Cam 观察点云与图像边缘对齐；Cam-Cam 观察对极线是否穿过对应点（对齐越好标定越准）。
5. **接受或取消**：
   - **Enter**：接受当前外参，写回结果并退出手动阶段。
   - **Esc**：取消手动调整，保留自动标定结果并退出。

（部分任务窗口内可能显示「S: 保存 / ESC: 接受并退出」，以**当前窗口或终端提示**为准。）

## 4. 按键说明（6-DOF 调整）

| 按键 | 功能 | 说明 |
|------|------|------|
| **Q / A** | Roll 增 / 减 | 绕 X 轴旋转 |
| **W / S** | Pitch 增 / 减 | 绕 Y 轴旋转 |
| **E / D** | Yaw 增 / 减 | 绕 Z 轴旋转 |
| **R / F** | Tx 增 / 减 | X 方向平移 |
| **T / G** | Ty 增 / 减 | Y 方向平移 |
| **Y / H** | Tz 增 / 减 | Z 方向平移 |
| **Shift + 上述键** | 10 倍步长 | 快速模式，同键位 |
| **U** | 撤销 (Undo) | 回退一步调整 |
| **Enter** | 接受 | 确认当前外参并退出手动阶段 |
| **Esc** | 取消 | 放弃手动调整，保留自动标定结果 |

- 默认步长（可配置）：旋转约 0.1°、平移约 0.005 m（0.5 cm）；快速模式约为 1°、0.05 m。
- 窗口内会实时显示当前 **T (m)** 与 **RPY (deg)**。LiDAR-Cam 看点云与图像边缘对齐；Cam-Cam 看左右图上的对极线是否合理（对齐越好标定越准）。

## 5. 可选配置项

- **手动会话**（`ManualCalibSession::SessionConfig`）
  - `enable_interactive_gui`：是否弹出 6-DOF 交互窗口（默认 `true`）。无图形界面时可设为 `false`，则仅保留自动结果。
  - `use_pangolin_manual_panel`：为 `true` 且编译时启用 Pangolin 时，LiDAR-Camera 手动校准使用 **Pangolin 点云叠加+左侧控制面板**（方案 B，参考 SensorsCalibration lidar2camera/manual_calib）：主视图为点云投影叠加图，左侧面板为 6-DOF ± 按钮、Intensity Color、Overlap Filter、点大小、Reset、Save Image、Accept；键盘 q/a,w/s,e/d,r/f,t/g,y/h 与 U/Enter/Esc 同上。
  - `save_dir`：会话保存目录（如 `output_dir/manual_sessions`）。
  - `auto_save`：是否在调整后自动保存会话。

- **流水线**（`PipelineConfig`）：`use_pangolin_manual_panel` 与上述一致，可由 YAML 或 `--pangolin-panel` 设置。

- **步长**（`ManualAdjustStep`）
  - `rot_step_deg`：旋转步长 [deg]，默认 0.1。
  - `trans_step_m`：平移步长 [m]，默认 0.005。
  - `rot_fast_deg` / `trans_fast_m`：快速模式步长（Shift 键）。

## 6. 数据流简述

1. **精标定阶段**：Pipeline 在 `run_fine_*` 成功后，会缓存 **首帧匹配数据**（仅第一帧：LiDAR 第 0 帧点云 + 相机第 0 帧图像，或双相机各第 0 帧图像 + 内参）到 `manual_cache_`。LiDAR-LiDAR 的首帧（ref 第 0 帧 + target 第 0 帧）可通过 `set_manual_lidar_lidar_first_frame()` 注入，或由未来的 `run_fine_lidar_lidar` 写入。
2. **手动阶段**：**手动调整仅使用上述首帧数据**，不加载多帧。当用户传入 **`--manual`**（即 `allow_manual_fallback` 为真）时，**对外参任务会始终进入手动阶段**；未传 `--manual` 时，仅当该任务 RMS 超过阈值才会进入。`run_manual_stage(task)` 会：
   - 从 `params_` 读取当前外参；
   - 从 `manual_cache_` 读取缓存的帧与内参；
   - 创建 `ManualCalibSession`，调用 `run_lidar_cam` / `run_cam_cam` / `run_lidar_lidar` / `run_imu_lidar`；
   - 用户在新窗口中按键调整，Enter 确认后将会话中的外参写回 `params_`。
3. **LiDAR-LiDAR**：与 LiDAR-Cam、Cam-Cam 一致，**手动标定也按首帧匹配数据**。当 `manual_cache_` 中存在 `lidar_lidar_ref_scan` 与 `lidar_lidar_target_scan` 时，会传入 `run_lidar_lidar(..., ref_first, target_first)` 并打日志「仅使用首帧匹配数据（ref 第 0 帧 + target 第 0 帧）」；未设置缓存时仍仅做 6-DOF 微调。

## 7. 注意事项与常见问题

- **图形界面**：使用 `--manual` 时，Pipeline 会启用手动可视化窗口。**默认**为 OpenCV 窗口：主视图为「点云叠加到图像」，可直接观察对齐效果。若使用 **`--pangolin-panel`**（或配置 `use_pangolin_manual_panel: true`），则 LiDAR-Camera 使用 Pangolin 窗口：主视图为点云叠加图、左侧为 6-DOF 与选项面板。需要本机或 Docker 具备可用 DISPLAY（脚本已配置 `-v /tmp/.X11-unix`、`-e DISPLAY`，宿主机需 `xhost +local:docker`）。无图形环境时窗口无法弹出，可改用无 GUI 模式运行（不加 `--manual` 或通过配置关闭交互）。
- **结果覆盖**：手动调整结果会覆盖该任务在 `params_` 中的外参，并随 Pipeline 结果一起保存到 YAML。
- **仅手动、不自动**：可先跑一次精标定生成初值，再在独立脚本中只调用 `ManualCalibSession::run_*` 进行微调并导出。
- **终端看不到说明**：运行 `./calib_unified_run.sh --run --task lidar-cam --manual` 时，在「运行前检查」框之后会打印「======== 手动标定操作说明 ========」及按键表；若未看到，可向上滚动终端或查看本文档。

## 8. 相关文档与脚本

- 脚本用法（含 `--manual` 示例）：见 `calib_unified_run.sh` 顶部注释及 `--task-help`。
- 外参质量与阈值：`EXTRINSIC_QUALITY_SCHEMA.md`。
