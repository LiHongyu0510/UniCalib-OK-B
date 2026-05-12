#pragma once
/**
 * UniCalib Unified — 3D 点云可视化 (替换 tiny-viewer)
 * 使用 PCL Visualizer 实现所有 3D 可视化功能
 * 完全移除 tiny-viewer 依赖
 */

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

// PCL 前向声明
namespace pcl {
  namespace visualization { class PCLVisualizer; }
  template<typename T> class PointCloud;
  struct PointXYZI;
  struct PointXYZRGB;
}

namespace ns_unicalib {

// ===================================================================
// 颜色结构体
// ===================================================================
struct ViewColor {
    float r = 1.f, g = 1.f, b = 1.f, a = 1.f;
    ViewColor() = default;
    ViewColor(float r_, float g_, float b_, float a_ = 1.f)
        : r(r_), g(g_), b(b_), a(a_) {}
    static ViewColor Red()    { return {1,0,0}; }
    static ViewColor Green()  { return {0,1,0}; }
    static ViewColor Blue()   { return {0,0,1}; }
    static ViewColor White()  { return {1,1,1}; }
    static ViewColor Yellow() { return {1,1,0}; }
    static ViewColor Cyan()   { return {0,1,1}; }
    static ViewColor Magenta(){ return {1,0,1}; }
    static ViewColor Gray()   { return {0.5f,0.5f,0.5f}; }
    // 生成独特颜色 (按索引)
    static ViewColor Unique(int idx);
};

// ===================================================================
// 点云查看器 (PCL Visualizer 封装)
// ===================================================================
class CloudViewer {
public:
    using Ptr = std::shared_ptr<CloudViewer>;
    using PointCloudI   = pcl::PointCloud<pcl::PointXYZI>;
    using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

    explicit CloudViewer(const std::string& window_title = "UniCalib Viewer",
                         bool background_thread = false);
    ~CloudViewer();

    // ---------------------------------------------------------------
    // 添加点云
    void add_cloud(const std::shared_ptr<PointCloudI>& cloud,
                   const std::string& id,
                   const ViewColor& color = ViewColor::White(),
                   float point_size = 1.0f);

    void add_cloud_rgb(const std::shared_ptr<PointCloudRGB>& cloud,
                       const std::string& id,
                       float point_size = 1.0f);

    // 带变换的点云 (自动着色)
    void add_cloud_with_pose(const std::shared_ptr<PointCloudI>& cloud,
                             const Sophus::SE3d& pose,
                             const std::string& id,
                             const ViewColor& color = ViewColor::White());

    // 多帧对齐点云 (用于 LiDAR-IMU 标定验证)
    void add_aligned_clouds(
        const std::vector<std::shared_ptr<PointCloudI>>& clouds,
        const std::vector<Sophus::SE3d>& poses,
        const std::string& prefix_id);

    // ---------------------------------------------------------------
    // 传感器可视化
    void add_coordinate_frame(const Sophus::SE3d& pose,
                               const std::string& id,
                               float scale = 0.3f);

    void add_sensor_poses(
        const std::map<std::string, Sophus::SE3d>& poses,
        float scale = 0.2f);

    // 相机锥体
    void add_camera_frustum(const Sophus::SE3d& pose,
                             double fx, double fy, double cx, double cy,
                             int width, int height,
                             const std::string& id,
                             const ViewColor& color = ViewColor::Blue(),
                             double scale = 0.3);

    // ---------------------------------------------------------------
    // 轨迹可视化
    void add_trajectory(const std::vector<Sophus::SE3d>& poses,
                        const std::string& id,
                        const ViewColor& color = ViewColor::Green(),
                        float line_width = 2.0f);

    // B样条轨迹 (密集采样)
    void add_trajectory_points(const std::vector<Eigen::Vector3d>& points,
                               const std::string& id,
                               const ViewColor& color = ViewColor::Yellow());

    // ---------------------------------------------------------------
    // 标定结果可视化
    // 显示 LiDAR-Camera 外参: 投影点云到图像平面
    void add_projected_cloud(const std::shared_ptr<PointCloudI>& cloud,
                              const Sophus::SE3d& T_lidar_to_cam,
                              double fx, double fy, double cx, double cy,
                              int width, int height,
                              const std::string& id);

    // ---------------------------------------------------------------
    // 控制
    void clear(const std::string& id = "");
    void remove(const std::string& id);
    void spin_once(int milliseconds = 100);
    void spin();     // 阻塞直到窗口关闭
    bool is_stopped() const;

    // 设置视角
    void set_camera_position(const Eigen::Vector3d& pos,
                              const Eigen::Vector3d& view_up = {0,0,1});

    // 截图
    void save_screenshot(const std::string& filename);

    // 异步更新回调 (后台线程模式)
    void set_update_callback(std::function<void(CloudViewer&)> cb);

    // 文字标注
    void add_text(const std::string& text, int x, int y,
                  const std::string& id, const ViewColor& color = ViewColor::White(),
                  int font_size = 16);

    // ---------------------------------------------------------------
    // 工厂方法
    static Ptr Create(const std::string& title = "UniCalib Viewer",
                      bool background = false) {
        return std::make_shared<CloudViewer>(title, background);
    }

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    std::thread viz_thread_;
    std::mutex  mtx_;
    std::atomic<bool> stopped_{false};
    std::function<void(CloudViewer&)> update_cb_;
};

// ===================================================================
// LiDAR 投影可视化 (LiDAR 点云投影到相机图像)
// ===================================================================
class LiDARProjectionViz {
public:
    // 将 LiDAR 点云投影到图像并保存
    static cv::Mat project_to_image(
        const pcl::PointCloud<pcl::PointXYZI>& cloud,
        const cv::Mat& image,
        const Sophus::SE3d& T_cam_in_lidar,
        double fx, double fy, double cx, double cy,
        int width, int height,
        double min_depth = 0.5,
        double max_depth = 50.0);
};

}  // namespace ns_unicalib
