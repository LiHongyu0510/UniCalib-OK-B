/**
 * UniCalib Unified — PCL 3D 点云可视化
 * 完全替代 tiny-viewer, 使用 PCL Visualizer
 */

#include "unicalib/viz/cloud_viewer.h"
#include "unicalib/common/logger.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <future>
#include <chrono>
#include <exception>

namespace ns_unicalib {

// ===================================================================
// ViewColor 独特颜色生成
// ===================================================================
ViewColor ViewColor::Unique(int idx) {
    // 使用 HSV 颜色空间均匀分布
    float h = std::fmod(idx * 137.508f, 360.0f);  // 黄金角度
    float s = 0.8f, v = 0.9f;
    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r_, g_, b_;
    if      (h < 60)  { r_ = c; g_ = x; b_ = 0; }
    else if (h < 120) { r_ = x; g_ = c; b_ = 0; }
    else if (h < 180) { r_ = 0; g_ = c; b_ = x; }
    else if (h < 240) { r_ = 0; g_ = x; b_ = c; }
    else if (h < 300) { r_ = x; g_ = 0; b_ = c; }
    else              { r_ = c; g_ = 0; b_ = x; }
    return ViewColor(r_ + m, g_ + m, b_ + m);
}

// ===================================================================
// CloudViewer 实现
// ===================================================================
CloudViewer::CloudViewer(const std::string& window_title, bool background_thread) {
    if (background_thread) {
        auto ready_promise = std::make_shared<std::promise<void>>();
        auto ready_future = ready_promise->get_future();

        viz_thread_ = std::thread([this, window_title, ready_promise]() {
            try {
                viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_title);
                viewer_->setBackgroundColor(0.1f, 0.1f, 0.1f);
                viewer_->addCoordinateSystem(0.5);
                viewer_->initCameraParameters();
                ready_promise->set_value();
            } catch (const std::exception& e) {
                UNICALIB_ERROR("[CloudViewer] Failed to initialize PCLVisualizer: {}", e.what());
                ready_promise->set_exception(std::current_exception());
                return;
            }

            while (!stopped_.load()) {
                {
                    std::unique_lock<std::mutex> lock(mtx_);
                    if (update_cb_) update_cb_(*this);
                    if (viewer_->wasStopped()) {
                        stopped_.store(true);
                        break;
                    }
                    viewer_->spinOnce(50);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        
        // 等待窗口初始化完成
        if (ready_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            ready_future.get();
        } else {
            UNICALIB_ERROR("[CloudViewer] PCLVisualizer initialization timeout (5s)");
        }
    } else {
        viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(window_title);
        viewer_->setBackgroundColor(0.1f, 0.1f, 0.1f);
        viewer_->addCoordinateSystem(0.5);
        viewer_->initCameraParameters();
    }
}

CloudViewer::~CloudViewer() {
    stopped_.store(true);
    if (viz_thread_.joinable()) viz_thread_.join();
    if (viewer_) viewer_->close();
}

// ===================================================================
// 添加点云 (灰度强度)
// ===================================================================
void CloudViewer::add_cloud(
    const std::shared_ptr<PointCloudI>& cloud,
    const std::string& id,
    const ViewColor& color,
    float point_size) {

    if (!cloud || cloud->empty()) return;
    std::lock_guard<std::mutex> lock(mtx_);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ch(
        cloud, color.r * 255, color.g * 255, color.b * 255);

    if (viewer_->contains(id)) {
        viewer_->updatePointCloud(cloud, ch, id);
    } else {
        viewer_->addPointCloud(cloud, ch, id);
    }
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

// ===================================================================
// 添加 RGB 点云
// ===================================================================
void CloudViewer::add_cloud_rgb(
    const std::shared_ptr<PointCloudRGB>& cloud,
    const std::string& id,
    float point_size) {

    if (!cloud || cloud->empty()) return;
    std::lock_guard<std::mutex> lock(mtx_);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> ch(cloud);
    if (viewer_->contains(id)) {
        viewer_->updatePointCloud(cloud, ch, id);
    } else {
        viewer_->addPointCloud(cloud, ch, id);
    }
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

// ===================================================================
// 带变换的点云
// ===================================================================
void CloudViewer::add_cloud_with_pose(
    const std::shared_ptr<PointCloudI>& cloud,
    const Sophus::SE3d& pose,
    const std::string& id,
    const ViewColor& color) {

    if (!cloud || cloud->empty()) return;

    // 变换点云
    auto cloud_transformed = std::make_shared<PointCloudI>();
    pcl::transformPointCloud(*cloud, *cloud_transformed,
                             Eigen::Affine3d(pose.matrix()));
    add_cloud(cloud_transformed, id, color);
}

// ===================================================================
// 多帧对齐点云
// ===================================================================
void CloudViewer::add_aligned_clouds(
    const std::vector<std::shared_ptr<PointCloudI>>& clouds,
    const std::vector<Sophus::SE3d>& poses,
    const std::string& prefix_id) {

    size_t N = std::min(clouds.size(), poses.size());
    for (size_t i = 0; i < N; ++i) {
        ViewColor color = ViewColor::Unique(static_cast<int>(i));
        add_cloud_with_pose(clouds[i], poses[i],
                            prefix_id + "_" + std::to_string(i), color);
    }
}

// ===================================================================
// 坐标轴
// ===================================================================
void CloudViewer::add_coordinate_frame(
    const Sophus::SE3d& pose,
    const std::string& id,
    float scale) {

    std::lock_guard<std::mutex> lock(mtx_);
    if (!viewer_) return;
    
    // PCL 某些版本中 addCoordinateSystem 不会自动覆盖同名 ID，需手动先移除
    try {
        viewer_->removeCoordinateSystem(id);
    } catch (...) {}

    Eigen::Affine3d T(pose.matrix());
    viewer_->addCoordinateSystem(scale, T.cast<float>(), id);
}

// ===================================================================
// 传感器位姿集合
// ===================================================================
void CloudViewer::add_sensor_poses(
    const std::map<std::string, Sophus::SE3d>& poses,
    float scale) {

    for (const auto& [id, pose] : poses) {
        add_coordinate_frame(pose, "frame_" + id, scale);
        // 添加文字标注
        Eigen::Vector3d pos = pose.translation();
        add_text(id,
                 static_cast<int>(pos[0] * 100),
                 static_cast<int>(pos[1] * 100),
                 "label_" + id);
    }
}

// ===================================================================
// 相机锥体
// ===================================================================
void CloudViewer::add_camera_frustum(
    const Sophus::SE3d& pose,
    double fx, double fy, double cx, double cy,
    int width, int height,
    const std::string& id,
    const ViewColor& color,
    double scale) {

    std::lock_guard<std::mutex> lock(mtx_);

    // 计算图像四角的 3D 射线方向
    std::vector<Eigen::Vector3d> corners = {
        {(0   - cx)/fx, (0    - cy)/fy, 1.0},
        {(width- cx)/fx, (0    - cy)/fy, 1.0},
        {(width- cx)/fx, (height- cy)/fy, 1.0},
        {(0   - cx)/fx, (height- cy)/fy, 1.0},
    };

    Eigen::Vector3d origin = pose.translation();
    cv::Scalar clr(color.b*255, color.g*255, color.r*255);

    // 绘制锥体边线
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d dir = pose.so3() * corners[i];
        Eigen::Vector3d tip = origin + dir * scale;

        pcl::PointXYZ p0(origin[0], origin[1], origin[2]);
        pcl::PointXYZ p1(tip[0], tip[1], tip[2]);
        viewer_->addLine(p0, p1, color.r, color.g, color.b,
                         id + "_line" + std::to_string(i));
    }
    // 底边
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        Eigen::Vector3d tip_i = origin + pose.so3() * corners[i] * scale;
        Eigen::Vector3d tip_j = origin + pose.so3() * corners[j] * scale;
        pcl::PointXYZ pi(tip_i[0], tip_i[1], tip_i[2]);
        pcl::PointXYZ pj(tip_j[0], tip_j[1], tip_j[2]);
        viewer_->addLine(pi, pj, color.r, color.g, color.b,
                         id + "_base" + std::to_string(i));
    }
}

// ===================================================================
// 轨迹可视化
// ===================================================================
void CloudViewer::add_trajectory(
    const std::vector<Sophus::SE3d>& poses,
    const std::string& id,
    const ViewColor& color,
    float line_width) {

    if (poses.size() < 2) return;
    std::lock_guard<std::mutex> lock(mtx_);

    for (size_t i = 1; i < poses.size(); ++i) {
        Eigen::Vector3d p0 = poses[i-1].translation();
        Eigen::Vector3d p1 = poses[i].translation();
        pcl::PointXYZ pt0(p0[0], p0[1], p0[2]);
        pcl::PointXYZ pt1(p1[0], p1[1], p1[2]);
        std::string line_id = id + "_seg" + std::to_string(i);
        viewer_->addLine(pt0, pt1, color.r, color.g, color.b, line_id);
        viewer_->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, line_id);
    }
}

void CloudViewer::add_trajectory_points(
    const std::vector<Eigen::Vector3d>& points,
    const std::string& id,
    const ViewColor& color) {

    if (points.empty()) return;
    auto cloud = std::make_shared<PointCloudI>();
    for (const auto& p : points) {
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(p[0]);
        pt.y = static_cast<float>(p[1]);
        pt.z = static_cast<float>(p[2]);
        pt.intensity = 1.0f;
        cloud->push_back(pt);
    }
    add_cloud(cloud, id, color, 2.0f);
}

// ===================================================================
// 控制
// ===================================================================
void CloudViewer::clear(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (id.empty()) {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
    } else {
        viewer_->removePointCloud(id);
        viewer_->removeShape(id);
    }
}

void CloudViewer::remove(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!viewer_) return;
    viewer_->removePointCloud(id);
    viewer_->removeShape(id);
    try {
        viewer_->removeCoordinateSystem(id);
    } catch (...) {}
}

void CloudViewer::spin_once(int milliseconds) {
    if (viewer_ && !viewer_->wasStopped()) {
        std::lock_guard<std::mutex> lock(mtx_);
        viewer_->spinOnce(milliseconds);
    }
}

void CloudViewer::spin() {
    while (!viewer_->wasStopped()) {
        if (update_cb_) {
            std::lock_guard<std::mutex> lock(mtx_);
            update_cb_(*this);
        }
        viewer_->spinOnce(100);
    }
    stopped_.store(true);
}

bool CloudViewer::is_stopped() const {
    return stopped_.load() || viewer_->wasStopped();
}

void CloudViewer::set_camera_position(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& view_up) {
    std::lock_guard<std::mutex> lock(mtx_);
    viewer_->setCameraPosition(
        pos[0], pos[1], pos[2],
        0, 0, 0,
        view_up[0], view_up[1], view_up[2]);
}

void CloudViewer::save_screenshot(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mtx_);
    viewer_->saveScreenshot(filename);
    UNICALIB_INFO("Screenshot saved: {}", filename);
}

void CloudViewer::set_update_callback(std::function<void(CloudViewer&)> cb) {
    std::lock_guard<std::mutex> lock(mtx_);
    update_cb_ = std::move(cb);
}

void CloudViewer::add_text(
    const std::string& text, int x, int y,
    const std::string& id,
    const ViewColor& color,
    int font_size) {
    std::lock_guard<std::mutex> lock(mtx_);
    viewer_->addText(text, x, y, font_size,
                     color.r, color.g, color.b, id);
}

// ===================================================================
// LiDAR 点云投影到图像
// ===================================================================
cv::Mat LiDARProjectionViz::project_to_image(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    const cv::Mat& image,
    const Sophus::SE3d& T_cam_in_lidar,
    double fx, double fy, double cx, double cy,
    int width, int height,
    double min_depth, double max_depth) {

    cv::Mat result;
    if (image.channels() == 1) {
        cv::cvtColor(image, result, cv::COLOR_GRAY2BGR);
    } else {
        result = image.clone();
    }

    // T_lidar_to_cam 是 LiDAR 到 Camera 的变换
    Sophus::SE3d T_lidar_to_cam = T_cam_in_lidar.inverse();

    for (const auto& pt : cloud.points) {
        Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);
        double depth = p_lidar.norm();
        if (depth < min_depth || depth > max_depth) continue;

        // 变换到相机坐标系
        Eigen::Vector3d p_cam = T_lidar_to_cam * p_lidar;
        if (p_cam[2] < min_depth) continue;  // 在相机后面

        // 投影
        double u = fx * p_cam[0] / p_cam[2] + cx;
        double v = fy * p_cam[1] / p_cam[2] + cy;

        if (u < 0 || u >= width || v < 0 || v >= height) continue;

        // 根据深度着色 (红近蓝远)
        double t = (depth - min_depth) / (max_depth - min_depth);
        t = std::max(0.0, std::min(1.0, t));
        int r = static_cast<int>((1.0 - t) * 255);
        int b = static_cast<int>(t * 255);
        int g = static_cast<int>(std::abs(0.5 - t) * 2 * 200);

        cv::circle(result, cv::Point(static_cast<int>(u), static_cast<int>(v)),
                   2, cv::Scalar(b, g, r), -1);
    }

    return result;
}

}  // namespace ns_unicalib
