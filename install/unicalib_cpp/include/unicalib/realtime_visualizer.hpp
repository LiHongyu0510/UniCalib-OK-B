/**
 * UniCalib C++ — 实时可视化器
 * 对应 Python: unicalib/utils/realtime_visualizer.py
 * 
 * 功能:
 * - 基于Open3D的实时3D渲染
 * - 点云动态更新
 * - 相机视锥可视化
 * - 坐标系可视化
 * - 轨迹线绘制
 */
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <unordered_map>

#ifdef UNICALIB_USE_OPEN3D
#include <open3d/Open3D.h>
#include <open3d/visualization/Visualizer.h>
#endif

#include <Eigen/Dense>

namespace unicalib {

/**
 * 点云几何体（用于Open3D）
 */
#ifdef UNICALIB_USE_OPEN3D
using PointCloudPtr = std::shared_ptr<open3d::geometry::PointCloud>;
using LineSetPtr = std::shared_ptr<open3d::geometry::LineSet>;
#endif

/** 实时可视化器 */
class RealtimeVisualizer {
 public:
  explicit RealtimeVisualizer(const std::string& window_name = "UniCalib Realtime Visualization");
  ~RealtimeVisualizer();

  /**
   * 检查可视化器是否可用
   */
  bool is_available() const;

  /**
   * 更新点云数据
   * @param name 点云名称（用于更新或创建）
   * @param points 点云坐标 (N×3)
   * @param colors 点云颜色 (N×3), 范围 [0,1]
   * @param point_size 点大小
   */
  void update_pointcloud(
      const std::string& name,
      const Eigen::MatrixXd& points,
      const Eigen::MatrixXd& colors = Eigen::MatrixXd(),
      double point_size = 2.0);

  /**
   * 更新相机视锥
   * @param name 视锥名称
   * @param R 旋转矩阵 (3×3)
   * @param t 平移向量 (3×1)
   * @param K 相机内参 (3×3)
   * @param image_size 图像尺寸 (width, height)
   * @param depth 视锥深度
   * @param color RGB 颜色 [0,1]
   */
  void update_frustum(
      const std::string& name,
      const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t,
      const Eigen::Matrix3d& K,
      const std::pair<int, int>& image_size,
      double depth = 5.0,
      const Eigen::Vector3d& color = Eigen::Vector3d(0.0, 1.0, 1.0));

  /**
   * 更新坐标系
   * @param name 坐标系名称
   * @param R 旋转矩阵 (3×3)
   * @param t 平移向量 (3×1)
   * @param scale 坐标轴长度
   */
  void update_coordinate_frame(
      const std::string& name,
      const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t,
      double scale = 1.0);

  /**
   * 添加轨迹线
   * @param name 轨迹名称
   * @param positions 轨迹点坐标 (N×3)
   * @param color 线条颜色 RGB [0,1]
   * @param line_width 线宽
   */
  void add_trajectory(
      const std::string& name,
      const Eigen::MatrixXd& positions,
      const Eigen::Vector3d& color = Eigen::Vector3d(1.0, 1.0, 0.0),
      double line_width = 2.0);

  /**
   * 清除几何体
   * @param name 要清除的几何体名称，nullptr表示全部清除
   */
  void clear_geometry(const char* name = nullptr);

  /**
   * 设置相机视角参数
   * @param front 前方向量
   * @param lookat 观察点
   * @param up 上方向量
   * @param zoom 缩放级别
   */
  void set_view_parameters(
      const Eigen::Vector3d& front = Eigen::Vector3d(0.0, 0.0, -1.0),
      const Eigen::Vector3d& lookat = Eigen::Vector3d(0.0, 0.0, 0.0),
      const Eigen::Vector3d& up = Eigen::Vector3d(0.0, -1.0, 0.0),
      double zoom = 0.5);

  /**
   * 关闭可视化器
   */
  void close();

 private:
  std::string window_name_;
  std::atomic<bool> running_;
  std::thread render_thread_;
  std::mutex mutex_;

  // Open3D 对象
#ifdef UNICALIB_USE_OPEN3D
  std::unique_ptr<open3d::visualization::Visualizer> vis_;
  
  // 几何体存储
  std::unordered_map<std::string, PointCloudPtr> pointclouds_;
  std::unordered_map<std::string, std::vector<LineSetPtr>> frustums_;
  std::unordered_map<std::string, std::vector<LineSetPtr>> coordinate_frames_;
  std::unordered_map<std::string, LineSetPtr> trajectories_;
#endif

  // 初始化方法
  void _try_init_open3d();

  // 渲染循环
  void _render_loop();

  // 处理队列中的更新项
  void _process_updates();
};

}  // namespace unicalib
