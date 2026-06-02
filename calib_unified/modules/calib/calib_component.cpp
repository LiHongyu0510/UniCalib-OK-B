#ifdef ENABLE_CYBER

#include "modules/calib/calib_component.h"
#include "unicalib/common/logger.h"
#include "proto/sync_frame.pb.h"
#include "proto/calib_status.pb.h"

#include "cyber/cyber.h"

namespace ns_unicalib {
namespace modules {

bool CalibComponent::Init() {
    pipeline_ = std::make_unique<CalibPipeline>();

    // 订阅同步帧
    CreateReader<unicalib::cyber::SyncFrame>(
        "/unicalib/sync/frame",
        [this](const std::shared_ptr<unicalib::cyber::SyncFrame>& msg) {
            // 将 SyncFrame 喂给 pipeline
            for (const auto& lidar : msg->lidars()) {
                // TODO: 反序列化点云
                // pipeline_->feed_lidar_frame(lidar.sensor_id(), ...);
            }
            for (const auto& cam : msg->cameras()) {
                // pipeline_->feed_camera_frame(cam.sensor_id(), ...);
            }
            for (const auto& imu : msg->imus()) {
                ImuData data;
                data.timestamp = imu.timestamp();
                // ... 填充
                pipeline_->feed_imu_data(data);
            }
        });

    // 状态发布
    status_writer_ = CreateWriter<unicalib::cyber::CalibStatus>("/unicalib/calib/status");

    UNICALIB_INFO("[CalibComponent] 初始化完成");
    return true;
}

bool CalibComponent::Proc() {
    // 定期尝试自动标定
    auto result = pipeline_->try_auto_calibrate(CalibTaskType::ALL);
    if (result.has_value() && result->success) {
        unicalib::cyber::CalibStatus status;
        status.set_status(unicalib::cyber::CalibStatus::SUCCESS);
        status.set_overall_rms(result->residual_rms);
        status_writer_->Write(status);
    }
    return true;
}

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER