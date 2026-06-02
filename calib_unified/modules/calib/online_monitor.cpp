#ifdef ENABLE_CYBER

#include "unicalib/common/logger.h"

namespace ns_unicalib {
namespace modules {

/**
 * 在线漂移检测模块（Stage 3）
 * 定期检查外参是否漂移，触发告警或自动重标定
 */
class OnlineMonitor {
public:
    void Start() {
        UNICALIB_INFO("[OnlineMonitor] 在线漂移检测已启动（占位实现）");
        // TODO: 滑动窗口 + 残差监控
        // TODO: 发布漂移告警到 /unicalib/calib/status
    }
};

} // namespace modules
} // namespace ns_unicalib

#endif // ENABLE_CYBER