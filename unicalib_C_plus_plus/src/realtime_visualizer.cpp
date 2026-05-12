/**
 * UniCalib C++ — RealtimeVisualizer 实现
 */
#include "unicalib/realtime_visualizer.hpp"
#include "unicalib/logger.hpp"
#include <algorithm>
#include <cmath>

#ifdef UNICALIB_USE_OPEN3D
#include <open3d/Open3D.h>
#endif

namespace unicalib {

RealtimeVisualizer::RealtimeVisualizer(const std::string& window_name)
    : window_name_(window_name), running_(false) {
  _try_init_open3d();
  
  if (running_) {
    // 启动渲染线程
    render_thread_ = std::thread(&RealtimeVisualizer::_render_loop, this);
    LOG_INFO("RealtimeVisualizer initialized successfully");
  }
}

RealtimeVisualizer::~RealtimeVisualizer() {
  close();
}

void RealtimeVisualizer::_try_init_open3d() {
#ifdef UNICALIB_USE_OPEN3D
  try {
    vis_ = std::make_unique<open3d::visualization::Visualizer>();
    vis_->CreateWindow(window_name_, 1280, 720);
    running_ = true;
    
    // 初始化几何体存储
#ifdef UNICALIB_USE_OPEN3D
    // geometries already initialized in header
#endif
    
    LOG_INFO("Open3D realtime visualizer initialized successfully");
    
  } catch (const std::exception& e) {
    std::string msg = "Failed to initialize Open3D: ";
    msg += e.what();
    LOG_ERROR(msg);
  }
#else
  running_ = false;
  LOG_WARNING("Open3D not available, realtime visualization disabled");
#endif
}

bool RealtimeVisualizer::is_available() const {
  return running_;
}

void RealtimeVisualizer::update_pointcloud(
    const std::string& name,
    const Eigen::MatrixXd& points,
    const Eigen::MatrixXd& colors,
    double point_size) {
  
  if (!is_available()) return;
  
#ifdef UNICALIB_USE_OPEN3D
  std::lock_guard<std::mutex> lock(mutex_);
  
  try {
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    
    // 转换点云坐标
    pcd->points_.resize(points.rows());
    for (int i = 0; i < points.rows(); ++i) {
      pcd->points_[i] = open3d::geometry::PointXYZ(
        points(i, 0), points(i, 1), points(i, 2));
    }
    
    // 设置颜色
    if (colors.rows() > 0 && colors.cols() >= 3) {
      pcd->colors_.resize(colors.rows());
      for (int i = 0; i < colors.rows(); ++i) {
        pcd->colors_[i] = open3d::geometry::PointXYZRGB(
          static_cast<uint8_t>(std::min(colors(i, 0), 1.0) * 255),
          static_cast<uint8_t>(std::min(colors(i, 1), 1.0) * 255),
          static_cast<uint8_t>(std::min(colors(i, 2), 1.0) * 255));
      }
    } else {
      // 默认深度着色
      pcd->colors_.resize(points.rows());
      std::vector<double> depths;
      depths.reserve(points.rows());
      for (int i = 0; i < points.rows(); ++i) {
        double depth = std::sqrt(points(i, 0) * points(i, 0) +
                               points(i, 1) * points(i, 1) +
                               points(i, 2) * points(i, 2));
        depths.push_back(depth);
      }
      
      std::vector<double> sorted_depths = depths;
      std::sort(sorted_depths.begin(), sorted_depths.end());
      double max_depth = sorted_depths[static_cast<size_t>(sorted_depths.size() * 0.95)] + 1e-6;
      
      for (int i = 0; i < points.rows(); ++i) {
        double norm_depth = std::min(depths[i] / max_depth, 1.0);
        pcd->colors_[i] = open3d::geometry::PointXYZRGB(
          static_cast<uint8_t>(norm_depth * 255),      // R
          static_cast<uint8_t>((1.0 - norm_depth) * 255), // G
          static_cast<uint8_t>(50));                      // B
      }
    }
    
    // 存储点云
    pointclouds_[name] = pcd;
    
    // 更新可视化器（在主线程中）
    vis_->RemoveGeometry(name);
    vis_->AddGeometry(*pcd);
    vis_->SetGeometryRenderingPointSize(name, point_size);
    
  } catch (const std::exception& e) {
    std::string msg = "Failed to update pointcloud " + name + ": ";
    msg += e.what();
    LOG_ERROR(msg);
  }
#endif
}

void RealtimeVisualizer::update_frustum(
    const std::string& name,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const Eigen::Matrix3d& K,
    const std::pair<int, int>& image_size,
    double depth,
    const Eigen::Vector3d& color) {
  
  if (!is_available()) return;
  
#ifdef UNICALIB_USE_OPEN3D
  std::lock_guard<std::mutex> lock(mutex_);
  
  try {
    // 相机原点
    Eigen::Vector3d origin = -R.transpose() * t;
    
    // 计算图像角点（相机坐标系）
    double fx = K(0, 0);
    double fy = K(1, 1);
    double cx = K(0, 2);
    double cy = K(1, 2);
    int w = image_size.first;
    int h = image_size.second;
    
    std::vector<Eigen::Vector3d> corners_cam = {
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(w, 0, 1),
      Eigen::Vector3d(w, h, 1),
      Eigen::Vector3d(0, h, 1)
    };
    
    // 变换到世界坐标系
    std::vector<Eigen::Vector3d> corners_world;
    for (const auto& corner : corners_cam) {
      Eigen::Vector3d pt = Eigen::Vector3d(corner(0) * depth / fx - depth * (cx - fx) / fx,
                                              corner(1) * depth / fy - depth * (cy - fy) / fy,
                                              depth);
      corners_world.push_back(origin + R.transpose() * pt);
    }
    
    // 创建线条
    std::vector<open3d::geometry::LineSet::LineType> lines;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> colors_vec;
    
    // 4条边线
    for (size_t i = 0; i < 4; ++i) {
      lines.push_back({static_cast<int>(points.size()), static_cast<int>(points.size() + 1)});
      points.push_back(origin);
      points.push_back(corners_world[i]);
      colors_vec.push_back(color);
      colors_vec.push_back(color);
    }
    
    // 底面4条边线
    for (size_t i = 0; i < 4; ++i) {
      lines.push_back({static_cast<int>(points.size()), static_cast<int>(points.size() + 1)});
      points.push_back(corners_world[i]);
      points.push_back(corners_world[(i + 1) % 4]);
      colors_vec.push_back(color);
      colors_vec.push_back(color);
    }
    
    // 创建LineSet
    auto line_set = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& pt : points) {
      line_set->points_.push_back({pt(0), pt(1), pt(2)});
    }
    line_set->lines_ = lines;
    
    line_set->colors_.resize(colors_vec.size());
    for (size_t i = 0; i < colors_vec.size(); ++i) {
      line_set->colors_[i] = open3d::geometry::PointXYZRGB(
        static_cast<uint8_t>(colors_vec[i](0) * 255),
        static_cast<uint8_t>(colors_vec[i](1) * 255),
        static_cast<uint8_t>(colors_vec[i](2) * 255));
    }
    
    // 存储并更新
    frustums_[name].clear();
    frustums_[name].push_back(line_set);
    
    vis_->RemoveGeometry(name + "_0");
    vis_->AddGeometry(*line_set);
    
  } catch (const std::exception& e) {
    std::string msg = "Failed to update frustum " + name + ": ";
    msg += e.what();
    LOG_ERROR(msg);
  }
#endif
}

void RealtimeVisualizer::update_coordinate_frame(
    const std::string& name,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    double scale) {
  
  if (!is_available()) return;
  
#ifdef UNICALIB_USE_OPEN3D
  std::lock_guard<std::mutex> lock(mutex_);
  
  try {
    // 坐标系原点
    Eigen::Vector3d origin = -R.transpose() * t;
    
    // 坐标轴终点
    Eigen::Vector3d x_end = origin + R.transpose().col(0) * scale;
    Eigen::Vector3d y_end = origin + R.transpose().col(1) * scale;
    Eigen::Vector3d z_end = origin + R.transpose().col(2) * scale;
    
    // 创建3条坐标轴线
    std::vector<open3d::geometry::LineSet::LineType> lines = {
      {0, 1},  // X轴
      {2, 3},  // Y轴
      {4, 5}   // Z轴
    };
    
    std::vector<Eigen::Vector3d> points = {
      origin, x_end,
      origin, y_end,
      origin, z_end
    };
    
    std::vector<Eigen::Vector3d> colors_vec = {
      Eigen::Vector3d(1.0, 0.0, 0.0),  // X: 红色
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),  // Y: 绿色
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),  // Z: 蓝色
      Eigen::Vector3d(0.0, 0.0, 1.0)
    };
    
    auto line_set = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& pt : points) {
      line_set->points_.push_back({pt(0), pt(1), pt(2)});
    }
    line_set->lines_ = lines;
    
    line_set->colors_.resize(colors_vec.size());
    for (size_t i = 0; i < colors_vec.size(); ++i) {
      line_set->colors_[i] = open3d::geometry::PointXYZRGB(
        static_cast<uint8_t>(colors_vec[i](0) * 255),
        static_cast<uint8_t>(colors_vec[i](1) * 255),
        static_cast<uint8_t>(colors_vec[i](2) * 255));
    }
    
    // 存储并更新
    coordinate_frames_[name].clear();
    coordinate_frames_[name].push_back(line_set);
    
    vis_->RemoveGeometry(name + "_0");
    vis_->AddGeometry(*line_set);
    
  } catch (const std::exception& e) {
    std::string msg = "Failed to update coordinate frame " + name + ": ";
    msg += e.what();
    LOG_ERROR(msg);
  }
#endif
}

void RealtimeVisualizer::add_trajectory(
    const std::string& name,
    const Eigen::MatrixXd& positions,
    const Eigen::Vector3d& color,
    double line_width) {
  
  if (!is_available()) return;
  
#ifdef UNICALIB_USE_OPEN3D
  std::lock_guard<std::mutex> lock(mutex_);
  
  try {
    // 创建连续的线条
    std::vector<open3d::geometry::LineSet::LineType> lines;
    lines.reserve(positions.rows() - 1);
    
    for (int i = 0; i < positions.rows() - 1; ++i) {
      lines.push_back({i, i + 1});
    }
    
    // 转换点云
    auto line_set = std::make_shared<open3d::geometry::LineSet>();
    for (int i = 0; i < positions.rows(); ++i) {
      line_set->points_.push_back({positions(i, 0), positions(i, 1), positions(i, 2)});
    }
    line_set->lines_ = lines;
    
    // 设置颜色
    line_set->colors_.resize(lines.size());
    for (size_t i = 0; i < lines.size(); ++i) {
      line_set->colors_[i] = open3d::geometry::PointXYZRGB(
        static_cast<uint8_t>(color(0) * 255),
        static_cast<uint8_t>(color(1) * 255),
        static_cast<uint8_t>(color(2) * 255));
    }
    
    // 设置线宽
    line_set->line_width_ = line_width;
    
    // 存储并更新
    trajectories_[name] = line_set;
    
    vis_->RemoveGeometry(name);
    vis_->AddGeometry(*line_set);
    
  } catch (const std::exception& e) {
    std::string msg = "Failed to add trajectory " + name + ": ";
    msg += e.what();
    LOG_ERROR(msg);
  }
#endif
}

void RealtimeVisualizer::clear_geometry(const char* name) {
  if (!is_available()) return;
  
#ifdef UNICALIB_USE_OPEN3D
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (name == nullptr) {
    // 清除所有几何体
    vis_->ClearGeometries();
    pointclouds_.clear();
    frustums_.clear();
    coordinate_frames_.clear();
    trajectories_.clear();
    LOG_INFO("Cleared all geometries");
  } else {
    // 清除指定几何体
    vis_->RemoveGeometry(std::string(name));
    
    // 从存储中删除
    pointclouds_.erase(name);
    frustums_.erase(name);
    coordinate_frames_.erase(name);
    trajectories_.erase(name);
    
    LOG_INFO("Cleared geometry: {}", name);
  }
#endif
}

void RealtimeVisualizer::set_view_parameters(
    const Eigen::Vector3d& front,
    const Eigen::Vector3d& lookat,
    const Eigen::Vector3d& up,
    double zoom) {
  
  if (!is_available()) return;
  
#ifdef UNICALIB_USE_OPEN3D
  std::lock_guard<std::mutex> lock(mutex_);
  
  try {
    auto& ctrl = vis_->GetViewControl();
    ctrl.SetFront({front(0), front(1), front(2)});
    ctrl.SetLookat({lookat(0), lookat(1), lookat(2)});
    ctrl.SetUp({up(0), up(1), up(2)});
    ctrl.SetZoom(zoom);
    
    LOG_INFO("Updated view parameters");
  } catch (const std::exception& e) {
    std::string msg = "Failed to set view parameters: ";
    msg += e.what();
    LOG_ERROR(msg);
  }
#endif
}

void RealtimeVisualizer::close() {
  running_ = false;
  
#ifdef UNICALIB_USE_OPEN3D
  if (vis_) {
    vis_->DestroyWindow();
  }
#endif
  
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
  
  LOG_INFO("RealtimeVisualizer closed");
}

void RealtimeVisualizer::_render_loop() {
  while (running_) {
#ifdef UNICALIB_USE_OPEN3D
    try {
      // 渲染一帧
      if (vis_ && vis_->PollEvents()) {
        vis_->UpdateRender();
      }
      
      // 短暂休眠（10ms）
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      
    } catch (const std::exception& e) {
      std::string msg = "Render loop error: ";
      msg += e.what();
      LOG_ERROR(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#else
    running_ = false;
    break;
#endif
  }
}

void RealtimeVisualizer::_process_updates() {
  // 处理队列中的更新（预留接口）
  // 当前实现直接在更新方法中调用Open3D API
}

}  // namespace unicalib
