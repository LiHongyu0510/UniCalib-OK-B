# DM-Calib 标定精度优化方案（基于最新研究）

## 0. 结论摘要（Executive Summary）

- **现状**：DM-Calib 为扩散模型单图/多图内参估计，在部分分辨率与场景下与 Kalibr 等几何标定偏差较大，精标定后仍可能不可用。
- **根因**：方法本身偏重零样本泛化与 3D 任务增强，并非为“替代高精度几何标定”设计；多图采用**均值**聚合易受离群帧影响；RANSAC/分辨率/去噪步数等可调空间未充分利用。
- **短期可落地**：提高质量参数（denoise_steps=20、ensemble_size=3、processing_res=768）、**多图取中位数**替代均值、增加参与推理帧数、优先使用棋盘格或参考标定做对比。
- **中期可选**：引入 DiffCalib/GeoCalib 等更新方法或后处理几何精化；对无棋盘格场景明确“仅作粗估，不保证工程精度”。
- **长期建议**：有精度要求时**以棋盘格/Kalibr 为准**；DM-Calib 作为无标定板时的初值或对比参考。

---

## 1. 最新研究中的方法与结论

### 1.1 DM-Calib（ICCV 2025）

- **思路**：Camera Image 表示 + 扩散生成 + RANSAC 提取内参。
- **优势**：零样本、增强深度/度量/位姿等 3D 任务。
- **局限**：论文未强调与几何标定（如 Kalibr）的逐像素级对齐；在部分分辨率/场景下误差较大属已知现象。

### 1.2 DiffCalib（AAAI 2025）

- **思路**：将标定重构为**稠密入射图（dense incident map）**的扩散生成，再用非学习的 RANSAC 从图中反推内参。
- **结论**：相较已有方法**预测误差可降低约 40%**；更充分利用预训练扩散模型的几何先验。
- **启示**：若需更高精度且可接入新模型，可评估 DiffCalib 替代或补充 DM-Calib。

### 1.3 GeoCalib（ECCV 2024）

- **思路**：单图标定 + **几何约束优化**（3D 几何规则 + 可微优化），并估计不确定性。
- **结论**：在鲁棒性与精度上优于纯学习与部分传统几何方法；**几何与学习结合**有利于精度。
- **启示**：在 DM-Calib 粗估基础上，用直线/消失点等几何做后处理精化，是可行的中期方向。

### 1.4 共性结论

| 方向           | 建议 |
|----------------|------|
| 多图融合       | 使用**中位数**等鲁棒聚合，减少单帧离群对结果的影响。 |
| 几何约束       | 引入直线/消失点/多视图约束可提升精度（GeoCalib、传统标定均如此）。 |
| 扩散质量参数   | 更高分辨率、更多去噪步、适当 ensemble 有利于稳定与精度，代价是算力。 |
| 结果使用策略   | 无棋盘格时明确“粗估”；有参考标定时用 reference_intrinsic_yaml 仅做对比，不替代结果。 |

---

## 2. 当前管线中的可调点

### 2.1 已暴露的配置（UniCalib）

| 参数                     | 默认   | 精度优先建议 | 说明 |
|--------------------------|--------|--------------|------|
| `dm_calib_denoise_steps` | 20     | 20（勿为加速降到 10 以下） | 去噪步数，影响扩散质量。 |
| `dm_calib_ensemble_size` | 3      | 3（勿为加速降到 1）       | 单图多次推理取平均。 |
| `dm_calib_processing_res`| 768    | 768（勿为加速降到 512 以下） | 推理分辨率，影响内参尺度一致性。 |
| 参与推理的图像数         | 15     | 提高至 20～30，并取**中位数** | 降低单帧离群影响。 |

### 2.2 脚本内部（DM-Calib）

- **多图聚合**：当前为 **mean**（均值），对离群敏感；改为 **median**（中位数）更稳。
- **RANSAC**（`tools/infer.py`）：`residual_threshold=1`、`max_trials=1000`；若后续开放为可配置，可针对场景微调。
- **畸变**：当前输出 k1=k2=p1=p2=0，无畸变项；对广角/鱼眼不适用，需依赖后续棋盘格或参考标定。

---

## 3. 短期可落地优化（本仓库内）

### 3.1 多图取中位数（已实现或待实现）

- 在 `infer_unicalib.py` 中，对多张图的 fx、fy、cx、cy 使用 **numpy.median** 聚合，替代 mean。
- 可选：通过 `--aggregation median|mean` 在“精度优先”与“兼容旧行为”间切换。

### 3.2 增加参与 DM-Calib 的帧数

- 在 `camera_intrinsic/main.cpp` 中，将 `max_for_dm` 从 15 提高到 20～30（可配置），使中位数更有统计意义。

### 3.3 配置与文档

- 在 `unicalib_example.yaml` 的 `third_party` 中注释明确：**精度优先时** denoise_steps=20、ensemble_size=3、processing_res=768，勿为加速再降低。
- 在 `CAMERA_INTRINSIC_ACCURACY.md` 中增加一节：DM-Calib 精度局限、何时用参考标定对比、何时必须用棋盘格。

---

## 4. 中期可选方向（需额外开发/依赖）

- **DiffCalib**：评估其推理接口与权重，若可封装为与现有 `infer_unicalib.py` 类似的 CLI，可作为可选后端（如 `third_party.dm_calib_backend: diffcalib`）。
- **GeoCalib**：作为 DM-Calib 的**后处理精化**：用 DM-Calib 输出为初值，在单图上做几何优化（直线/消失点），得到更稳的 fx/fy/cx/cy。
- **RANSAC 可配置**：在 DM-Calib 的 `calculate_intrinsic` 处暴露 `residual_threshold` / `max_trials` 为命令行或环境变量，便于针对数据集调参。

---

## 5. 使用建议（何时信任 DM-Calib 结果）

| 场景               | 建议 |
|--------------------|------|
| 有棋盘格/标定板     | **以棋盘格标定结果为最终结果**，DM-Calib 仅作无棋盘格时的 fallback 或对比。 |
| 有 Kalibr 等参考   | 使用 `reference_intrinsic_yaml` 仅做**精度对比**，不替代输出；用于评估 DM-Calib 偏差。 |
| 无棋盘格、无参考   | 将 DM-Calib 视为**粗估**；若下游对精度敏感，应补采棋盘格或换用几何标定。 |
| 分辨率/场景与论文差异大 | 偏差可能较大；优先提高 processing_res、denoise_steps、ensemble_size，并用多图中位数。 |

---

## 6. 参考文献与链接

- DM-Calib: [GitHub](https://github.com/JunyuanDeng/DM-Calib), ICCV 2025.
- DiffCalib: [arXiv:2405.15619](https://arxiv.org/html/2405.15619), AAAI 2025, ~40% 误差降低。
- GeoCalib: [arXiv:2409.06704](https://arxiv.org/html/2409.06704), ECCV 2024, 几何优化单图标定。
