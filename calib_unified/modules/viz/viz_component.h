#ifdef ENABLE_CYBER

#pragma once

#include "cyber/component/component.h"
#include "cyber/cyber.h"

namespace ns_unicalib {
namespace modules {

/**
 * VizComponent
 * 负责实时可视化：
 * - 点云 + 相机投影叠加
 * - 外参残差曲线
 * - 标定状态面板
 */
class VizComponent : public apollo::cyber::Component<> {
public:
    bool Init() override;
    bool Proc() override;

private:
    // TODO: Pangolin 或 OpenCV 可视化窗口
    // TODO: 订阅 /unicalib/sync/frame 和 /unicalib/calib/status
};

DECLARE_COMPONENT(VizComponent);

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER