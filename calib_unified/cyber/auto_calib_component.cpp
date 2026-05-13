// ============================================================
// AutoCalibComponent 实现（预留，未编译）
// ============================================================
// 说明：
//   本文件实现了 Cyber 自动标定组件的完整骨架。
//   目前所有关键逻辑都用占位或 TODO 标记，取消 #if 0 后即可逐步填充。
// ============================================================

// #if 0   // <--- 取消这行即可编译本文件

#include "cyber/auto_calib_component.h"
#include "unicalib/common/logger.h"

namespace ns_unicalib {
namespace cyber {

bool AutoCalibComponent::Init() {
    // 1. 读取配置文件（支持 YAML + Cyber 参数）
    config_path_ = DeclareParameter<std::string>("config_file", "");
    if (config_path_.empty()) {
        UNICALIB_ERROR("[Cyber] config_file 参数未设置");
        return false;
    }

    // 2. 初始化标定流水线（复用现有 CalibPipeline）
    // pipeline_ = std::make_unique<CalibPipeline>(pipeline_cfg_);

    // 3. 初始化共享内存通道（可选）
    // shm_channel_ = std::make_unique<ShmCalibrationChannel>("/unicalib_calib", true);

    // 4. 创建 Cyber Reader（订阅 LiDAR、Camera、IMU）
    // lidar_reader_ = CreateReader<apollo::cyber::proto::PointCloud2>(
    //     "lidar_topic", [this](const auto& msg){ /* feed_frame */ });

    UNICALIB_INFO("[Cyber] AutoCalibComponent 初始化完成，等待数据...");
    return true;
}

bool AutoCalibComponent::Proc() {
    // Cyber 10.0.0 中 Proc 通常留空，数据通过 Reader 回调进入
    // 此处可放定时检查逻辑
    static uint64_t tick = 0;
    if (++tick % 100 == 0) {
        try_auto_calibrate();
    }
    return true;
}

void AutoCalibComponent::try_auto_calibrate() {
    if (!should_trigger_calibration()) {
        return;
    }

    UNICALIB_INFO("[Cyber] 触发自动标定...");

    // TODO: 调用 pipeline_->run() 或 feed 版本
    // auto result = pipeline_->run(CalibTaskType::ALL);

    // if (result.success) {
    //     publish_result(result);
    // }
}

bool AutoCalibComponent::should_trigger_calibration() const {
    // TODO: 实现静止检测、时间间隔、特征丰富度判断
    // 例如：
    // if (vehicle_speed > 0.3) return false;
    // if (time_since_last_calib < 600s) return false;
    return false;   // 默认不触发，待实现
}

void AutoCalibComponent::publish_result(const StageResult& /*result*/) {
    // 1. 通过 Cyber Writer 发布 CalibrationResult proto
    // 2. 同时更新 ShmCalibrationChannel（零拷贝给主机进程）
    // shm_channel_->publish(...);
}

} // namespace cyber
} // namespace ns_unicalib

// #endif // UNICALIB_ENABLE_CYBER