# 点云估计传感器距地高度 — 方法说明

脚本路径：`scripts/estimate_sensor_height_from_pcd.py`

## 1. 目的与原则

- **目标**：从激光点云估计各传感器（顶雷达、补盲雷达、IMU）相对**拟合地面**的距地高度，以及顶雷达相对地面的倾角。
- **不使用安装表**：车体 V 系、安装表 Y 高度等不参与计算。
- **各雷达独立**：每个雷达使用**本传感器 PCD**、在**本传感器坐标系**内单独拟合地面并算高度；不用合并点云、不用主雷达地面去算补盲高度。

外参文件 `calibration_param.yaml` 仅用于 **IMU 位置**（`T_imu_0__lidar_main`）。

---

## 2. 输入与数据对齐

| 输入 | 说明 |
|------|------|
| `lidar_main` PCD | 主雷达单帧，本体系 |
| `left_front` PCD | 左前补盲单帧，本体系 |
| `right_back` PCD | 右后补盲单帧，本体系 |
| `calibration_param.yaml` | IMU–主雷达外参（可选） |
| `merged_lidar_main/merged_idx0.pcd` | 三路雷达融合点云（**IMU 地面拟合**，lidar_main 系） |

**对齐方式**（三帧需对应同一时刻）：

- `--pcd-main / --pcd-left / --pcd-right` 手动指定；或
- `--align-time`：按文件名时间戳（ms）对齐；或
- `--frame-index N`：各目录排序后第 N 帧。

**示例**：

```bash
python3 scripts/estimate_sensor_height_from_pcd.py \
  --pcd-main  data/lidar_main/top_front_0519/1779177302500.pcd \
  --pcd-left  data/lidar_airy/20260519/left_front/1779177302500.pcd \
  --pcd-right data/lidar_airy/20260519/right_back/1779177302500.pcd

python3 scripts/estimate_sensor_height_from_pcd.py --align-time
```

---

## 3. 坐标系约定

各传感器本地轴见 `.cursor/rules/sensor-frame-conventions.mdc`：

| 传感器 | 轴记法 | 脚本中「向上」向量 `z_up` | 路面点在 Z 上的特征 |
|--------|--------|---------------------------|---------------------|
| `lidar_main` | 前左上，Z 上 | `(0, 0, +1)` | 路面为 **低 Z** |
| `left_front` | 前右下，Z 下 | `(0, 0, -1)` | 路面为 **高 Z** |
| `right_back` | 后左下，Z 下 | `(0, 0, -1)` | 路面为 **高 Z** |

**默认路面 ROI**（本体系，单位 m）：

| 传感器 | X 范围 | \|Y\| ≤ |
|--------|--------|--------|
| 顶雷达 | [3, 24] | 4 |
| 左前补盲 | [-10, 4] | 4 |
| 右后补盲 | [-10, 5] | 4 |

补盲雷达前方路面常落在较小/负的 X 区间，故与顶雷达 ROI 不同。主雷达可用 `--x-min` / `--x-max` 覆盖。

---

## 4. 地面拟合（每雷达独立）

在**该雷达**当帧 PCD 的 ROI 内执行：

### 4.1 地面候选点

1. 按 Z 筛选路面一侧：
   - Z 向上：保留 Z ≤ 约 18% 分位 + 0.10 m；
   - Z 向下：保留 Z ≥ 约 82% 分位 − 0.10 m。
2. **XY 网格**（默认 0.25 m），每格取极值点（Z 上取最低，Z 下取最高）。
3. **平坦性过滤**：剔除局部粗糙度过大、沿 X 坡度过大的格子（抑制路沿、障碍）。

### 4.2 平面方程

对网格种子点 **最小二乘** 拟合平面：

\[
\mathbf{n} \cdot \mathbf{p} + d = 0
\]

法向 **n** 与该雷达的 `z_up` 同向（指向上方）。

### 4.3 迭代精化

在 ROI 内取近地面点，距平面 < 约 4.5 cm 的保留，重复拟合 3 次。输出地面拟合 **RMS**（典型约 10–13 mm）。

---

## 5. 距地高度定义

传感器原点取雷达坐标系原点 **(0, 0, 0)**。

### 5.1 地面高度

在原点水平位置 \((x_0, y_0)\) 处，地平面上的 Z：

\[
z_{\text{ground}} = -\frac{n_x x_0 + n_y y_0 + d}{n_z}
\]

### 5.2 竖直距地高度 H

沿该雷达「向上」方向的竖直高度：

| 类型 | 公式 |
|------|------|
| Z 向上（顶雷达） | \(H = z_0 - z_{\text{ground}}\) |
| Z 向下（补盲） | \(H = z_{\text{ground}} - z_0\) |

统一向量形式：

\[
H = \mathbf{z}_{\text{up}} \cdot (\mathbf{p}_{\text{sensor}} - \mathbf{p}_{\text{ground}})
\]

其中 \(\mathbf{p}_{\text{ground}} = (x_0, y_0, z_{\text{ground}})\)。

### 5.3 顶雷达倾角

顶雷达 **Z 轴** 与地平面法向 **n** 的夹角（度）。另输出近似俯仰 pitch、横滚 roll。

---

## 6. IMU 距地高度

IMU 无激光点云：**地面来自融合点云，位置来自外参**。详细汇报说明见 [第 11 节](#11-imu-距地高度汇报版)。

| 项目 | 计算方法 |
|------|----------|
| **IMU** | `T_imu_0__lidar_main` 约定 `p_lidar = T·p_imu`（与 `lidar_main__*` 一致），IMU 在顶雷达系位置 = **平移列 `translation`**；在融合点云 `merged_idx0.pcd` 上拟合地面后算竖直高度。 |

汇总中的 **「Δ 相对主雷达」** = 该传感器绝对高度 − 顶雷达高度。因顶雷达与 IMU **地面拟合数据源不同**（单帧 PCD vs 融合 PCD），此差值仅供参考，不完全等于外参里的竖直距离。

---

## 11. IMU 距地高度（汇报版）

本节面向汇报/评审，用尽量直白的语言说明 **IMU 高度怎么算、用了什么数、结果怎么读**。

### 11.1 要算什么

**IMU 距地高度**：IMU 中心（IMU 坐标系原点）到**路面**的竖直距离，单位 mm。

IMU 没有点云，因此：

- **地面** → 由激光点云拟合；
- **IMU 在哪里** → 由外参文件给出。

### 11.2 用了哪些数据

| 项目 | 文件 | 作用 |
|------|------|------|
| 融合点云 | `merged_lidar_main/merged_idx0.pcd` | 三路雷达已对齐到 **顶雷达坐标系**，用于拟合前方路面 |
| IMU–顶雷达外参 | `merged_lidar_main/calibration_param.yaml` → `T_imu_0__lidar_main` | 给出 IMU 原点在顶雷达系下的位置 |
| 顶雷达高度 | 顶雷达**单帧** PCD 单独计算 | 仅用于汇总表「Δ 相对主雷达」 |

**不使用安装表**，不读取 `results/` 下其它标定输出。

### 11.3 坐标系（汇报时建议说明一句）

- 全程在 **顶雷达坐标系 lidar_main** 下计算（前–左–上，**Z 轴向上**）。
- 外参与补盲雷达一致：

\[
\mathbf{p}_{\text{顶雷达}} = T \cdot \mathbf{p}_{\text{IMU}}
\]

- **IMU 原点在顶雷达系下的坐标** = 矩阵 **`translation` 平移列**（不要用 `inv(T)` 的平移，那是另一种约定下的结果）。

当前示例外参（米）：

```text
translation: [0.0160, -0.0304, -0.3048]
```

含义：IMU 在顶雷达系中约在 **前 16 mm、左 30 mm、下 305 mm**（Z 为负表示低于顶雷达原点）。

### 11.4 计算三步

#### 步骤 1：在融合点云上拟合地面

1. 取车前方路面：**x ∈ [3, 24] m**，**|y| ≤ 4 m**（与顶雷达地面 ROI 一致）。
2. 在 ROI 内按网格取各格**最低点**作为地面候选（Z 向上，路面在下方）。
3. 拟合平面 \(\mathbf{n} \cdot \mathbf{p} + d = 0\)（**n** 大致竖直向上）。
4. 迭代剔除离平面过远的点，重新拟合（典型残差 RMS 约 10–20 mm）。

**直观理解**：在融合点云里用前方马路拟合一个地面平面。

#### 步骤 2：确定 IMU 在顶雷达系下的位置

从 `calibration_param.yaml` 读取 `T_imu_0__lidar_main`（3×3 旋转 R + 平移 t）：

\[
\mathbf{p}_{\text{顶雷达}} = R \cdot \mathbf{p}_{\text{IMU}} + \mathbf{t}
\]

IMU 原点（IMU 系下为 0）在顶雷达系下就是 **t**：

\[
\mathbf{p}_{\text{IMU,顶雷达}} = \texttt{translation}
\]

**汇报要点**：IMU 位置完全由该外参决定；外参更新后，IMU 高度会随之变化。

#### 步骤 3：算竖直距地高度

在 IMU 的 \((x, y)\) 处求地面高度，再算竖直差（详见 [11.4.1](#1141-得到-imu-位置后如何算竖直距地)）。

\[
z_{\text{ground}}(x, y) = -\frac{n_x x + n_y y + d}{n_z}, \qquad
H_{\text{IMU}} = z_{\text{IMU}} - z_{\text{ground}}(x_{\text{IMU}}, y_{\text{IMU}})
\]

**直观理解**：在 IMU 正下方 \((x_{\text{IMU}}, y_{\text{IMU}})\) 处查地面 Z，再用 IMU 的 Z 减去该地面 Z。

#### 11.4.1 得到 IMU 位置后，如何算竖直距地

步骤 1、2 完成后，已有：

| 量 | 含义 |
|----|------|
| \(\mathbf{n}, d\) | 融合点云拟合的地面平面：\(n_x x + n_y y + n_z z + d = 0\) |
| \(\mathbf{p} = (x, y, z)\) | IMU 原点在顶雷达系下的坐标（外参 `translation`） |

**算法不是**「点到平面的法向垂直距离」，而是 **在 IMU 的水平坐标处取地面 Z，再沿顶雷达竖直轴（Z 向上）做高度差**。

##### ① 在 \((x, y)\) 处求地面高度

把 IMU 的 \(x, y\) 代入平面方程，解出该位置地面点的 \(z\)：

\[
z_{\text{ground}} = -\frac{n_x x + n_y y + d}{n_z}
\]

脚本函数 `z_ground_at_xy`；要求 \(|n_z|\) 不能太小（地面不能近似竖直于 XY）。

##### ② 竖直距地高度

\[
H_{\text{IMU}} = z - z_{\text{ground}}
\]

脚本函数 `vertical_height`：先算 \(z_{\text{ground}}\)，再算 \(\mathbf{z}_{\text{up}} \cdot (\mathbf{p} - (0, 0, z_{\text{ground}}))\)。顶雷达系下 \(\mathbf{z}_{\text{up}} = (0, 0, 1)\)，故结果就是 **\(z - z_{\text{ground}}\)**。

结果单位为 **米**，打印汇总时 **× 1000** 得到 mm。

##### ③ 数值示例（示意）

| 量 | 值 |
|----|-----|
| \(z_{\text{IMU}}\) | −0.305 m（外参，IMU 在顶雷达原点下方） |
| \(z_{\text{ground}}(x_{\text{IMU}}, y_{\text{IMU}})\) | 约 −1.39 m（由拟合平面在该 (x,y) 处算出） |
| \(H_{\text{IMU}}\) | \((-0.305) - (-1.39) \approx 1.08\) m **≈ 1083 mm** |

含义：从 IMU 沿 **竖直向上** 到脚下地面的高度约 1.08 m。

##### ④ 与「法向距离」的区别

沿平面法向的 signed distance 为 \((\mathbf{n} \cdot \mathbf{p} + d) / \|\mathbf{n}\|\)。路面近似水平时，与竖直高度差很小；坡道较大时，本脚本仍用 **竖直差**，与顶雷达、补盲雷达定义一致，便于横向对比。

##### ⑤ 小结（两步）

```text
输入：平面 (n, d)  +  IMU 位置 p = (x, y, z)

① z_地 = -(n_x·x + n_y·y + d) / n_z
② H    = z − z_地        →  距地高度（m）
```

### 11.5 流程图

```text
融合点云（顶雷达系） ──► 前方 ROI 拟合地面平面 (n, d)
                                    │
外参 T_imu_0__lidar_main ──► IMU 位置 t（translation）
                                    │
                                    ▼
              H_IMU = Z_IMU − Z_地面(x_IMU, y_IMU)
```

### 11.6 典型结果示例（便于写进汇报）

以下为同一套数据、默认参数下的一次运行量级（具体数值随点云帧略有波动）：

| 项目 | 数值 |
|------|------|
| IMU 距地高度 | **约 1083 mm** |
| 顶雷达距地高度 | **约 1418 mm** |
| Δ 相对顶雷达 | **约 −334 mm**（IMU 低于顶雷达约 33 cm） |
| 融合点云地面拟合 RMS | **约 17 mm** |

**自检**：外参 Z 方向 IMU 比顶雷达低约 305 mm，与「Δ 相对顶雷达」约 −334 mm 量级一致（差额来自两边地面拟合略有不同）。

### 11.7 与顶雷达高度的关系

| 项目 | 顶雷达 | IMU |
|------|--------|-----|
| 点云 | 顶雷达**单帧** PCD | 无（用外参定位） |
| 地面 | 本帧 PCD 在顶雷达系拟合 | **融合点云**在顶雷达系拟合 |
| 传感器位置 | 原点 (0,0,0) | 外参 `translation` |

汇总表 **「Δ 相对主雷达」= H_IMU − H_顶雷达**，是两个**绝对距地高度**的差，不是直接从外参读出的高度差。

### 11.8 前提与局限（汇报可带一句）

1. 前方需有足够平坦路面（直道、平坡更稳）。
2. IMU 高度 = **点云地面** + **外参位置**；外参误差会直接进入结果。
3. 顶雷达与 IMU 使用不同点云做地面拟合，绝对高度可能有厘米级差异。
4. 结果为**几何距地高度**，与车体 V 系安装表测量零点无关。

### 11.9 一句话结论（可直接引用）

> **IMU 距地高度**：在顶雷达坐标系下，用三路融合点云拟合前方路面得到地面；再用 `calibration_param.yaml` 中 IMU–顶雷达外参的平移项确定 IMU 位置；最后计算 IMU 原点相对该地面的竖直距离。

---

## 7. 外参语义（calibration_param.yaml）

| 键 | 含义 | 用途 |
|----|------|------|
| `lidar_main__left_front` | `p_main = T · p_left`，平移 = 左前原点在主雷达系 | 本脚本**不用于**补盲高度（补盲用本机 PCD） |
| `lidar_main__right_back` | 同上 | 同上 |
| `T_imu_0__lidar_main` | `p_lidar = T · p_imu` | IMU 原点在主雷达系：**`translation`**（勿用 `inv(T)[:3,3]`） |

**注意**：IMU 外参**仅**使用 `merged_lidar_main/calibration_param.yaml` 中的 `T_imu_0__lidar_main`，不读取 `results/` 下其它标定输出。

---

## 8. 流程示意

```
主雷达 PCD ──► ROI 拟合地面 ──► H_main

融合 PCD (main 系) ──► ROI 拟合地面 n,d ──► IMU: translation → H_imu

左前 PCD ──► 本体系 ROI ──► 拟合地面 ──► H = z_ground - z   (Z 向下)

右后 PCD ──► 本体系 ROI ──► 拟合地面 ──► H = z_ground - z   (Z 向下)
```

---

## 9. 精度与局限

**适用**

- 前方有足够平坦路面；平坡、直道场景更稳。

**误差来源**

- 路沿、坡道、遮挡污染地面点；
- 三雷达各自拟合地面，若路面高低/倾角不一致，绝对高度与「Δ 相对主雷达」不能严格等同外参推导的 ΔZ；
- IMU 依赖主雷达地面 + IMU–主雷达外参精度。

**不适用**

- 用安装表直接当距地高度；
- 用合并点云统一拟合地面代替单雷达（除非只分析主雷达）。

---

## 10. 相关文件

| 文件 | 说明 |
|------|------|
| `scripts/estimate_sensor_height_from_pcd.py` | 实现脚本 |
| `merged_lidar_main/calibration_param.yaml` | 默认外参路径 |
| `scripts/merge_three_lidars_to_main.py` | 三路点云合并（与本高度算法独立） |
| `calib_unified/docs/LIDAR_LIDAR_CALIB_COMPUTATION_AND_ACCURACY.md` | 雷达–雷达外参约定 |
