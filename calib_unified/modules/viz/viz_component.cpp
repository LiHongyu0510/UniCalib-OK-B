#ifdef ENABLE_CYBER

#include "modules/viz/viz_component.h"
#include "unicalib/common/logger.h"

namespace ns_unicalib {
namespace modules {

bool VizComponent::Init() {
    UNICALIB_INFO("[VizComponent] 初始化完成（Stage 3 占位实现）");
    // TODO: 初始化 Pangolin / OpenCV 窗口
    // TODO: 创建 Reader 订阅 SyncFrame 和 CalibStatus
    return true;
}

bool VizComponent::Proc() {
    // TODO: 实时渲染点云投影 + 残差曲线
    return true;
}

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER