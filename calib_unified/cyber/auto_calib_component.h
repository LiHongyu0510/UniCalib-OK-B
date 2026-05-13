// ============================================================
// AutoCalibComponent — Cyber RT 自动标定组件（预留）
// ============================================================
// 继承 cyber::Component，实现：
//   - 订阅多传感器话题（PointCloud2、Image、Imu）
//   - 后台自动触发标定（静止检测 + 定时器）
//   - 结果通过共享内存 + Cyber topic 发布
// ============================================================
// 启用方法：
//   1. 在 cyber/ 目录下实现本组件
//   2. 在 CMakeLists.txt 中加入编译并链接 Cyber
//   3. 编写 .dag 文件供 cyber_launch 使用
// ============================================================

#pragma once

// #if 0   // <--- 取消注释后启用

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/common/calib_param.h"
#include "cyber/shm_calibration_channel.h"

#include <memory>
#include <string>
#include <vector>

namespace ns_unicalib {
namespace cyber {

class AutoCalibComponent : public apollo::cyber::Component<> {
public:
    bool Init() override;
    bool Proc() override;   // Cyber 10.0.0 中 Proc 可留空，主要用 Reader 回调

private:
    // 配置
    std::string config_path_;
    PipelineConfig pipeline_cfg_;

    // 核心标定引擎
    std::unique_ptr<CalibPipeline> pipeline_;

    // 共享内存通道（可选）
    std::unique_ptr<ShmCalibrationChannel> shm_channel_;

    // Cyber 读者/写者
    // std::shared_ptr<apollo::cyber::Reader<...>> lidar_reader_;
    // std::shared_ptr<apollo::cyber::Writer<CalibrationResult>> result_writer_;

    // 自动触发逻辑
    void try_auto_calibrate();
    bool should_trigger_calibration() const;

    // 结果发布
    void publish_result(const StageResult& result);
};

DECLARE_COMPONENT(AutoCalibComponent);

} // namespace cyber
} // namespace ns_unicalib

// #endif // UNICALIB_ENABLE_CYBER