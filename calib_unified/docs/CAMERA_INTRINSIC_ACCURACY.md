# 相机内参标定 — 如何提高精度

## 0. 结论摘要

- **最高精度**：使用**棋盘格/标定板**采集足够多、多姿态图像，让结果走 **opencv_chessboard**，并以 RMS 评估。
- **无棋盘格时**：用 **DM-Calib** 且提高其质量参数（denoise_steps/ensemble_size/processing_res），或提供准确先验。
- **配置**：增大 `min_images`、收紧 `max_rms_px`、保证 `target.square_size` 与实物一致，DM-Calib 用“精度优先”参数。

---

## 1. 两种结果来源与精度

| 结果来源 | 说明 | 精度与评估 |
|----------|------|------------|
| **opencv_chessboard** | 棋盘格角点 + OpenCV 标定 | 高；有重投影 RMS (px)，可复现 |
| **dm_calib** | 无棋盘格时 DM-Calib 网络估计 | 中等；无棋盘格时无 RMS，依赖网络与参数 |
| **prior** | 仅先验/默认内参 | 最低，仅作占位 |

**参考标定对比**：配置 `data.camera.<id>.reference_intrinsic_yaml`（如 Kalibr 的 YAML 路径，相对 `--data-dir`）时，程序在得到标定结果后**仅与该文件做精度对比**（打日志、在 accuracy CSV 中记录 ref_* / diff_*），**不参与、不替代输出结果**。输出结果仍为本次 DM-Calib/棋盘格标定所得。

**建议**：若需要高精度，尽量**有棋盘格数据**，并保证有效帧数 ≥ `min_images`，使最终采用 `opencv_chessboard` 结果。基于最新研究的**棋盘格标定算法优化**（亚像素参数、离群帧剔除、优化终止条件等）见 **[CHECKERBOARD_CALIBRATION_RESEARCH.md](CHECKERBOARD_CALIBRATION_RESEARCH.md)**。

---

## 2. 提高精度的配置与数据

### 2.1 棋盘格标定（优先）

- **min_images**：建议 **25～40**（默认 15）。有效帧越多、姿态越多样，标定越稳。
- **max_images**：可保持 **80～100**，保证覆盖多角度、多距离，避免只用少数几帧。
- **max_rms_px**：默认 1.5；精度优先可设为 **1.0**，超过会报警，便于筛掉质量差的标定。
- **refine_corners**：保持 **true**（亚像素角点），已从配置读取。
- **target**：
  - **cols / rows**：与标定板**内角点**数一致（非格子数）。
  - **square_size**：与实物**格子边长 (m)** 严格一致，测量误差会直接进内参。
- **采集建议**：
  - 覆盖画面四角与中心、多距离（近/远）、多角度（倾斜、正对）。
  - 避免模糊、过曝、反光；光照均匀。

### 2.2 无棋盘格时（DM-Calib）

DM-Calib 为扩散模型粗估，在部分分辨率/场景下与几何标定（如 Kalibr）偏差可能较大，**精标定后若仍不可用属已知局限**。优化思路见 **[DM-Calib 标定精度优化方案](DM_CALIB_ACCURACY_OPTIMIZATION.md)**（最新研究、多图中位数、可选 DiffCalib/GeoCalib）。

在 `third_party` 中**精度优先**时建议（勿为加速再降低）：

```yaml
# 精度优先：不配或设为 0 则用脚本/默认
dm_calib_denoise_steps: 20    # 默认 20；勿降到 10 以下
dm_calib_ensemble_size: 3     # 默认 3；勿降到 1
dm_calib_processing_res: 768  # 默认 768；勿降到 512 以下
dm_calib_max_images: 25       # 参与推理的图像数，多图取中位数聚合（默认 25）
```

### 2.3 策略选择

- **prefer_no_checkerboard: true**（默认）：无棋盘格不失败，用 DM-Calib/先验；**有**棋盘格且有效帧足够时仍会用棋盘格结果并优先采用（RMS 更可靠）。
- **prefer_no_checkerboard: false**：强制仅棋盘格标定；无棋盘格或有效帧不足会直接失败。适合“只做高精度棋盘格”的场景。

---

## 3. 配置项速查

| 配置项 | 默认 | 精度优先建议 |
|--------|------|----------------|
| camera_intrinsic.min_images | 15 | 25～40 |
| camera_intrinsic.max_images | 100 | 80～100，且保证姿态多样 |
| camera_intrinsic.max_rms_px | 1.5 | 1.0 |
| camera_intrinsic.refine_corners | true | 保持 true |
| camera_intrinsic.rational_model | false | 边缘畸变大时可试 true（k4～k6） |
| camera_intrinsic.max_rms_px（精度优先） | 1.5 | 1.0 或 0.5，便于发现退化 |
| camera_intrinsic.target.square_size | 0.025 | 与实物一致 (m) |
| third_party.dm_calib_denoise_steps | (脚本 20) | 20 |
| third_party.dm_calib_ensemble_size | (脚本 3) | 3 |
| third_party.dm_calib_processing_res | (脚本 768) | 768 |
| third_party.dm_calib_max_images | 25 | 20～30（多图中位数更稳） |

---

## 4. 验证与排查

- 日志中查看 **method**：`opencv_chessboard` 表示本次采用棋盘格结果。
- 棋盘格结果会打印 **重投影 RMS=… px**；RMS &lt; max_rms_px 且越小越好。
- 若始终为 `dm_calib`：检查图像中是否可见完整棋盘格、cols/rows 是否与标定板一致、有效帧是否 ≥ min_images。
- 若 RMS 偏大：增加有效帧、检查 square_size、改善采集（清晰、多姿态、光照）。

---

## 5. 当前 RMS 已较低（如 &lt;0.3 px）时进一步压低

你当前结果 **RMS=0.1882 px** 已属很好（&lt;0.5 通常为优秀）。若仍希望再压低或保持稳定，可做：

| 措施 | 说明 |
|------|------|
| **收紧 max_rms_px** | 设为 **1.0** 或 **0.5**，超过即报警，便于发现退化。 |
| **增加有效帧数** | 将 **min_images** 提到 **25～40**，并补充更多**多姿态**图像（四角、倾斜、远近），标定更稳。 |
| **核对 square_size** | 用尺子量标定板一格边长，填准 **target.square_size**（单位 m），误差会直接进内参。 |
| **rational_model** | 在 `camera_intrinsic` 下设 **rational_model: true**，使用 k4～k6 有理畸变，边缘畸变大时可能略降 RMS；部分视觉库不支持 k4+，按需开启。 |
| **数据质量** | 剔除模糊、过曝、反光帧；保证光照均匀、标定板平整。 |
