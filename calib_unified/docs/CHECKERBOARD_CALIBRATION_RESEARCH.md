# 基于棋盘格的相机标定 — 最新研究与可落地优化

## 0. 结论摘要

- **研究趋势**：亚像素角点精化、离群帧剔除（RANSAC/σ 阈值）、混合靶标（棋盘格+圆点）、更紧的优化终止条件，均可提升标定精度。
- **本仓库已支持/本次增强**：亚像素参数可配置、标定后按重投影误差剔除离群帧并重新标定、有理畸变模型可选、优化迭代次数/精度可配置。
- **建议**：在保证 `square_size` 与实物一致、多姿态多帧的前提下，开启 `refine_corners`，适当收紧亚像素与优化参数，并启用离群帧剔除（如 `outlier_rejection_sigma: 2.0`）。

---

## 1. 最新研究中的结论

### 1.1 亚像素角点（Subpixel Refinement）

- **作用**：将角点从像素级细化到亚像素，可明显降低重投影误差；对去马赛克、光学特性引起的偏差有补偿作用。
- **OpenCV**：`cv::cornerSubPix`，常用参数：
  - **winSize**：搜索窗口，如 (11,11) 或略大；
  - **criteria**：终止条件，研究与实践常用 **迭代次数 50**、**精度 0.001**（更紧于默认 30/0.01）。
- **本仓库**：`refine_corners: true` 时调用 `cornerSubPix`；可通过 `subpix_max_iter`、`subpix_epsilon` 调节（精度优先建议 50、0.001）。

### 1.2 离群帧剔除（Outlier Rejection）

- **研究结论**：标定中部分帧角点定位误差大或存在运动模糊/遮挡，会拉高整体 RMS；用 RANSAC 或“均值+σ 标准差”剔除离群帧后重新标定，可提高鲁棒性与精度。
- **常见做法**：首次标定后计算每帧重投影误差，剔除误差 &gt; mean + k×std（如 k=2）的帧，在剩余帧上再次标定。
- **本仓库**：可选 `outlier_rejection_sigma`（如 2.0），标定完成后自动剔除超阈值帧并重新标定，保留帧数仍满足 `min_images` 时才生效。

### 1.3 混合靶标与新型图案

- **Hybrid Chessboard–Circle（2025）**：在棋盘格上叠加圆点，用棋盘角点做透视校正后再提取圆心，报告相比传统棋盘格重投影误差约 **降低 51%**（平均约 0.0365 px）。需要专用靶标与检测流程。
- **PuzzleBoard（2024）**：带位置编码的新图案，在低分辨率下仍可提供更多有效点。需更换标定板与检测算法。
- **本仓库**：当前为经典棋盘格 + 可选圆点网格；若未来引入混合靶标，可在同一 pipeline 下增加一种 target 类型。

### 1.4 优化终止条件与畸变模型

- **calibrateCamera** 的 `TermCriteria`：迭代次数与 ε 越紧，解越收敛，但耗时略增；精度优先时可提高迭代次数（如 100→200）、收紧 ε（如 1e-7→1e-8）。
- **有理畸变（rational_model）**：启用 k4～k6，边缘畸变较大时可能进一步降低 RMS；部分下游仅支持 k1,k2,p1,p2，需按需开启。

---

## 2. 本仓库中的可配置项（精度优先建议）

| 配置项 | 默认 | 精度优先建议 | 说明 |
|--------|------|----------------|------|
| camera_intrinsic.refine_corners | true | 保持 true | 亚像素角点 |
| camera_intrinsic.subpix_max_iter | 30 | 50 | 亚像素迭代次数 |
| camera_intrinsic.subpix_epsilon | 0.01 | 0.001 | 亚像素收敛精度 |
| camera_intrinsic.outlier_rejection_sigma | 0 | 2.0 | 离群帧剔除（0=关闭）；剔除误差>mean+σ×std 的帧后重新标定 |
| camera_intrinsic.calibrate_max_iter | 100 | 200 | 标定优化最大迭代次数 |
| camera_intrinsic.calibrate_epsilon | 1e-7 | 1e-8 | 标定优化收敛精度 |
| camera_intrinsic.rational_model | false | 按需 true | 有理畸变 k4～k6 |
| camera_intrinsic.max_rms_px | 1.5 | 1.0 或 0.5 | 报警阈值 |
| camera_intrinsic.target.square_size | — | 与实物严格一致 | 用尺子测量 |

---

## 3. 数据与采集建议

- **有效帧数**：≥ 25～40，姿态多样（四角、中心、倾斜、远近）。
- **标定板**：平整、无反光；格子边长用尺子量准后填入 `square_size`。
- **图像质量**：避免模糊、过曝；光照均匀。
- **离群**：启用 `outlier_rejection_sigma` 后，程序会自动剔除误差过大的帧并重新标定，无需手动删图。

---

## 4. 参考文献与链接

- OpenCV: [Detecting corners in subpixels](https://docs.opencv.org/4.x/dd/d92/tutorial_corner_subpixels.html)
- Practical refinement: [Camera Calibration: Practical OpenCV Refinement Techniques](https://nikolasent.github.io/computervision/opencv/calibration/2024/12/20/Practical-OpenCV-Refinement-Techniques.html)
- RANSAC/outlier: Accurate and robust estimation of camera parameters using RANSAC (ScienceDirect)
- Hybrid target: High-precision camera calibration via perspective distortion correction using a hybrid chessboard–circle target (Optica, 2025)
- PuzzleBoard: [PuzzleBoard: A New Camera Calibration Pattern with Position Encoding](https://arxiv.org/html/2409.20127v1)
