#ifdef ENABLE_CYBER

#pragma once

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "proto/sync_frame.pb.h"

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ns_unicalib {
namespace modules {

/**
 * SyncComponent
 * 负责多传感器时间同步 + 组帧
 * Stage 2 基础实现：简单时间窗口聚合
 */
class SyncComponent : public apollo::cyber::Component<> {
public:
    bool Init() override;
    bool Proc() override;

private:
    void OnLidar(const std::shared_ptr<apollo::cyber::proto::PointCloud2>& msg);
    void OnCamera(const std::shared_ptr<apollo::cyber::proto::Image>& msg);
    void OnImu(const std::shared_ptr<apollo::cyber::proto::Imu>& msg);

    void TryPublishSyncFrame();

    std::mutex mutex_;
    // 简单缓存（Stage 2 用 map 模拟）
    std::map<std::string, std::vector<std::shared_ptr<void>>> lidar_cache_;
    std::map<std::string, std::vector<std::shared_ptr<void>>> camera_cache_;
    std::vector<std::shared_ptr<void>> imu_cache_;

    std::shared_ptr<apollo::cyber::Writer<unicalib::cyber::SyncFrame>> sync_writer_;
};

DECLARE_COMPONENT(SyncComponent);

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER