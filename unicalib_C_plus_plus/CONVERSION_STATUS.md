# UniCalib Python → C++ 转换完成度分析

**本项目以 unicalib_C_plus_plus（C++）为主框架编译和运行，不使用 UniCalib Python。**

对照 **Readme.mk** 文档、**UniCalib** 源码与 **unicalib_C_plus_plus** 实现，整理如下。

---

## 一、Readme.mk 与目录一致性

- **Readme.mk 六、工程构建指南**（约 1748–1832 行）描述的是 **UniCalib** 的 **Python** 工程结构，目录写为 `UniCalib/src/...`（如 `src/core/system.py`）。
- **实际 UniCalib 仓库**顶层为 `unicalib/`（无 `src/`），即 `unicalib/core/system.py`、`unicalib/intrinsic/` 等，与文档略有差异，但模块名一致。
- **Readme.mk 未提及** `unicalib_C_plus_plus` 或 “Python 转 C++” 的说明；C++ 侧仅在 `unicalib_C_plus_plus/README.md` 有说明。

---

## 二、Python 模块与 C++ 转换对照表

| 序号 | UniCalib Python 模块 | unicalib_C_plus_plus 对应 | 状态 | 说明 |
|------|----------------------|----------------------------|------|------|
| **core** |
| 1 | `core/sensor_config.py` | `sensor_config.hpp/cpp` | ✅ 已完成 | SensorType, SensorConfig, CalibPair, auto_infer_calib_pairs |
| 2 | `core/calib_result.py` | `calib_result.hpp/cpp` | ✅ 已完成 | CameraIntrinsic, FisheyeIntrinsic, IMUIntrinsic, CalibResult, ValidationReport |
| 3 | `core/data_manager.py` | `data_manager.hpp/cpp` | ⚠️ 部分 | 目录模式 + CSV IMU 已实现；**rosbag2 未实现** |
| 4 | `core/system.py` | `system.hpp/cpp` | ⚠️ 部分 | 配置解析、四阶段框架；Stage1 用 IMUIntrinsicCalibrator（Allan+六面）；保存时写 YAML + ReportGenerator；Stage2/3/4 占位 |
| **utils** |
| 5 | `utils/transforms.py` | `transforms.hpp/cpp` | ✅ 已完成 | 欧拉角、四元数、SE3、鲁棒旋转平均、插值 |
| 6 | `utils/visualization.py` | — | ❌ 未转换 | 可视化逻辑未移植 |
| 7 | `utils/ros_utils.py` | — | ❌ 未转换 | ROS 工具未移植 |
| **intrinsic** |
| 8 | `intrinsic/allan_variance.py` | `allan_variance.hpp/cpp` | ✅ 已完成 | 简化版 OADEV（等价 Python _simple_analysis），无 allantools |
| 9 | `intrinsic/imu_intrinsic.py` | `imu_intrinsic.hpp/cpp` | ✅ 已完成 | Allan + 六面法简化 + build_from_allan；不调用 iKalibr/Transformer |
| 10 | `intrinsic/camera_pinhole.py` | `dm_calib_wrapper.hpp/cpp` | ✅ 已实现 | DM-Calib 子进程调用 + JSON 解析；OpenCV 精化未移植 |
| 11 | `intrinsic/camera_fisheye.py` | `camera_fisheye.hpp/cpp` | ✅ 已实现 | OpenCV fisheye (equidistant)，可选编译 `UNICALIB_USE_OPENCV`；Python 仍保留多模型 EUCM/DS/KB |
| **extrinsic/coarse** |
| 12 | `extrinsic/coarse/learn_to_calib_wrapper.py` | `learn_to_calib_wrapper.hpp/cpp` | ✅ 已实现 | learn-to-calibrate 子进程调用 + YAML 解析（IMU-LiDAR 粗外参） |
| 13 | `extrinsic/coarse/imu_lidar_init.py` | — | ❌ 未转换 | IMU-LiDAR 初始化未移植 |
| 14 | `extrinsic/coarse/feature_matching.py` | — | ❌ 未转换 | 特征匹配未移植 |
| **extrinsic/fine** |
| 15 | `extrinsic/fine/ikalibr_wrapper.py` | — | ❌ 未转换 | iKalibr 封装未移植 |
| 16 | `extrinsic/fine/mias_lcec_wrapper.py` | — | ❌ 未转换 | MIAS-LCEC 封装未移植 |
| **validation** |
| 17 | `validation/reprojection.py` | `reprojection.hpp/cpp` | ⚠️ 占位 | 接口已有；无图像/点云时返回占位指标 |
| 18 | `validation/colorization.py` | — | ❌ 未转换 | 点云着色验证未移植 |
| 19 | `validation/edge_alignment.py` | — | ❌ 未转换 | 边缘对齐未移植 |
| 20 | `validation/click_calib_wrapper.py` | — | ❌ 未转换 | click_calib 封装未移植 |
| 21 | `validation/report_generator.py` | `report_generator.hpp/cpp` | ✅ 已完成 | 终端 print_summary + HTML 报告；保存时写 YAML |
| **其他** |
| 22 | `ros2_node.py` | — | ❌ 未转换 | ROS2 节点未移植 |
| 23 | `scripts/*.py`, `launch/*.py`, `tests/*` | `examples/example_main.cpp` 等 | ⚠️ 部分 | 仅有 C++ 示例入口，无对等脚本/launch/测试 |

---

## 三、结论：是否“完成转换”

- **按“全部功能对等”**：**未完成**。  
  已转换：**核心数据结构、几何变换、Allan 方差、IMU 内参（Allan+六面法）、数据管理（目录/CSV）、报告生成（终端+HTML+YAML）、重投影验证接口（占位）、系统主控与保存流程**。  
  未移植：相机内参（针孔/鱼眼）、外参粗/精算法、完整重投影/着色/边缘验证、click_calib、ROS/可视化。

- **按“核心数据与流程骨架可复现”**：**已完成**。  
  配置解析、标定对推断、IMU 内参标定与保存、结果 YAML 与 HTML 报告、四阶段调用顺序均已实现；外参与部分验证为占位，可后续接算法。

建议在 **Readme.mk** 中增加一小节，说明 C++ 工程位置与当前完成度，便于与文档一致。
