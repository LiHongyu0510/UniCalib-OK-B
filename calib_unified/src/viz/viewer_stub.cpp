/**
 * UniCalib 可视化 Stub 实现
 * 用于无 GUI 场景（Headless / 纯计算）
 * 
 * 当编译选项关闭所有可视化后端时，此文件提供无操作实现
 */

#include "ikalibr/viewer/viewer.h"

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ns_ikalibr {

/**
 * Stub 实现：所有可视化方法为空操作 (no-op)
 * - 不创建任何 GUI 窗口
 * - 不占用 GPU 资源
 * - 用于服务器/CI-CD 环境
 */
class ViewerImpl : public std::enable_shared_from_this<ViewerImpl> {
public:
    ViewerImpl() = default;
    ~ViewerImpl() = default;

    static Ptr Create(const std::string& title) {
        return std::make_shared<Viewer>(title);
    }

    // ─────────────────────────────────────────────────────
    // 所有可视化方法为空实现 (no-op)
    // ─────────────────────────────────────────────────────
    
    void AddPointCloud(const VetaPtr& veta) {
        // no-op
    }
    
    void AddTrajectory(const SplineBundlePtr& spline) {
        // no-op
    }
    
    void ShowTrajectories(const std::string& topic, const Colour& colour) {
        // no-op
    }
    
    void ClearViewer() {
        // no-op
    }
    
    void Spin() {
        // no-op：无 GUI，直接返回
    }
    
    void SpinOnce(int ms = 100) {
        // no-op：无 GUI，直接返回
    }
    
    bool WaitKey(int ms = -1) {
        // 总是返回 false
        return false;
    }
};

}  // namespace ns_ikalibr
