#ifdef ENABLE_CYBER

#include "modules/sync/sync_component.h"
#include "unicalib/common/logger.h"

namespace ns_unicalib {
namespace modules {

bool SyncComponent::Init() {
    // TODO: 从配置或参数加载 topic 名称
    // 创建 Writer
    sync_writer_ = CreateWriter<unicalib::cyber::SyncFrame>("/unicalib/sync/frame");

    // TODO: 创建 Reader（示例）
    // CreateReader<apollo::cyber::proto::PointCloud2>(
    //     "/lidar/points", std::bind(&SyncComponent::OnLidar, this, std::placeholders::_1));

    UNICALIB_INFO("[SyncComponent] 初始化完成");
    return true;
}

bool SyncComponent::Proc() {
    // 定期尝试发布同步帧
    TryPublishSyncFrame();
    return true;
}

void SyncComponent::OnLidar(const std::shared_ptr<apollo::cyber::proto::PointCloud2>& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 简化：按 sensor_id 缓存
    lidar_cache_["default"].push_back(msg);
    if (lidar_cache_["default"].size() > 5) lidar_cache_["default"].erase(lidar_cache_["default"].begin());
}

void SyncComponent::OnCamera(const std::shared_ptr<apollo::cyber::proto::Image>& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_cache_["default"].push_back(msg);
}

void SyncComponent::OnImu(const std::shared_ptr<apollo::cyber::proto::Imu>& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_cache_.push_back(msg);
    if (imu_cache_.size() > 200) imu_cache_.erase(imu_cache_.begin());
}

void SyncComponent::TryPublishSyncFrame() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (lidar_cache_["default"].empty()) return;

    unicalib::cyber::SyncFrame frame;
    frame.set_timestamp_ns(0); // TODO: 取真实时间
    frame.set_frame_id("sync_" + std::to_string(frame.timestamp_ns()));

    // TODO: 真正填充 lidar / camera / imu 数据

    sync_writer_->Write(frame);
    UNICALIB_DEBUG("[SyncComponent] 发布 SyncFrame");
}

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER