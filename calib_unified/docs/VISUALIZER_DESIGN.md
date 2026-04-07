# UniCalib 可视化工具设计方案

## 1. 概述

### 1.1 设计目标

基于Pangolin设计一个通用的可视化工具，实现：
1. 实时显示标定过程和结果
2. 支持手动调整外参（多种旋转表示）
3. 显示标定精度（重投影误差等）
4. 兼容现有手动校准模块

### 1.2 技术选型

| 组件 | 技术 | 理由 |
|------|------|------|
| 3D渲染 | Pangolin | 项目已有依赖，轻量级 |
| UI面板 | Pangolin Var/Panel | 内置参数调节 |
| 2D显示 | OpenCV imshow | 图像显示 |
| 交互 | Pangolin Handler | 3D交互 |

## 2. 系统架构

### 2.1 模块划分

```
┌─────────────────────────────────────────────────────────┐
│                    CalibVisualizer                      │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │
│  │  3D View   │  │  2D View   │  │ Control Panel   │  │
│  │ (Pangolin) │  │ (OpenCV)   │  │ (Pangolin)      │  │
│  └─────────────┘  └─────────────┘  └─────────────────┘  │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────┐│
│  │           VisualizerData (共享数据结构)            ││
│  └─────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
```

### 2.2 旋转调整模式

由于欧拉角存在万向节锁问题，设计支持多种旋转表示：

| 模式 | 参数 | 优点 | 缺点 |
|------|------|------|------|
| EULER | roll/pitch/yaw | 直观简单 | 万向节锁 |
| AXIS_ANGLE | axis(x,y,z) + angle | 无奇点，直观 | 不连续 |
| QUATERNION | w/x/y/z | 无奇点，数值稳定 | 不直观 |
| ROTATION_VECTOR | (rx,ry,rz) | 与BA优化兼容 | 不直观 |

**推荐**: AXIS_ANGLE模式（直观且无奇点）

## 3. 功能设计

### 3.1 实时可视化

**3D视图功能:**
- 显示参考点云（绿色）
- 显示目标点云（蓝色）
- 显示配准后点云（红色）
- 显示相机视锥体
- 显示坐标系轴
- 网格平面

**2D投影功能:**
- LiDAR点云投影到相机图像
- 重投影误差向量（颜色编码）
- 误差统计显示

### 3.2 手动外参调整

**键盘控制:**
```
平移:
  W/S - Z轴前后 (+/-)
  A/D - X轴左右 (-/+)
  Q/E - Y轴上下 (+/-)

旋转 (当前模式):
  I/K - Roll   (+/-)
  J/L - Pitch (+/-)
  U/O - Yaw   (+/-)

其他:
  R - 重置到初始外参
  Enter - 确认并保存
  Esc - 取消并退出
  Tab - 切换旋转模式
  +/- - 调整步长
```

**轴角模式交互:**
- 鼠标左键拖拽：绕-view轴旋转
- 鼠标右键拖拽：平移
- 滚轮：缩放

### 3.3 标定精度显示

**状态栏信息:**
```
状态: [阶段] | RMS: X.XX px | 收敛: Yes | 进度: XX%
```

**颜色编码:**
- 绿色: RMS < 1.0 px (优秀)
- 黄色: 1.0 < RMS < 3.0 px (良好)
- 红色: RMS > 3.0 px (需调整)

## 4. 与现有模块集成

### 4.1 集成方式

```cpp
// 在标定流程中集成可视化
void calibrate_with_visualization() {
    CalibVisualizer::Ptr viz = create_visualizer(config);
    viz->init("LiDAR-Camera Calibration");
    
    // 设置回调
    viz->set_on_accept([&]() {
        // 保存最终外参
        save_extrinsic(viz->get_extrinsic());
    });
    
    // 主循环
    while (calibrating && viz->is_running()) {
        // 获取当前数据
        auto data = get_current_data();
        
        // 更新可视化
        viz->update(data);
        
        // 渲染
        viz->render_once();
    }
}
```

### 4.2 与ManualCalib集成

将可视化器作为ManualCalib的后端，保留原有交互逻辑。

## 5. 运行方式

```bash
# 编译后运行
./calib_unified_run.sh --build-only
./calib_unified_run.sh --run --task lidar-cam --visualize

# 或在代码中启用
CalibVisualizerConfig cfg;
cfg.window_width = 1920;
cfg.window_height = 1080;
cfg.default_rot_step = 0.5;  // 0.5度步长
cfg.default_trans_step = 0.01;  // 1cm步长

auto viz = create_visualizer(cfg);
```

## 6. 参考文献

1. **Pangolin**: https://github.com/stevenlovegrove/Pangolin
2. **lidar-camera-calibration-manual**: https://github.com/fwan133/lidar-camera-calibration-manual
3. **direct_visual_lidar_calibration**: https://github.com/koide3/direct_visual_lidar_calibration
4. **SuperGlue** (CVPR 2020): 学习特征匹配

## 7. 后续工作

1. 完成Pangolin渲染实现（当前为存根）
2. 集成到标定pipeline
3. 添加键盘/鼠标交互
4. 添加重投影误差热力图
5. 支持视频录制
