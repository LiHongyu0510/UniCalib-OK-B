/**
 * UniCalib C++ — VisualizationV2 实现
 */
#include "unicalib/visualization_v2.hpp"
#include "unicalib/calib_result.hpp"
#include "unicalib/transforms.hpp"
#include "unicalib/logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>

#ifdef UNICALIB_USE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

namespace unicalib {

VisualizationV2::VisualizationV2(const std::string& output_dir)
    : output_dir_(output_dir) {
  // 确保输出目录存在
  std::string mkdir_cmd = "mkdir -p " + output_dir_;
  system(mkdir_cmd.c_str());
}

#ifdef UNICALIB_USE_OPENCV
cv::Mat VisualizationV2::DrawLidarHeatmap(
    const cv::Mat& img,
    const Eigen::MatrixXd& pts_cam,
    const Eigen::Matrix3d& K,
    const Eigen::Vector4d& D,
    const VisualizationConfig& config) {
  
  if (img.empty() || pts_cam.rows() == 0) {
    return img.clone();
  }

  cv::Mat result;
  img.clone().convertTo(result, CV_32FC3);
  int H = img.rows;
  int W = img.cols;

  // 过滤正深度的点
  std::vector<int> valid_indices;
  for (int i = 0; i < pts_cam.rows(); ++i) {
    if (pts_cam(i, 2) > 0.1) {
      valid_indices.push_back(i);
    }
  }

  if (valid_indices.empty()) {
    return img.clone();
  }

  // 投影到图像平面
  std::vector<cv::Point2f> pts_2d;
  std::vector<double> depths;
  
  for (int idx : valid_indices) {
    double x = pts_cam(idx, 0) / pts_cam(idx, 2);
    double y = pts_cam(idx, 1) / pts_cam(idx, 2);
    
    double r2 = x*x + y*y;
    double radial = 1.0 + D(0)*r2 + D(1)*r2*r2 + D(2)*r2*r2*r2;
    
    double x_distorted = x * radial;
    double y_distorted = y * radial;
    
    double u = K(0,0)*x_distorted + K(0,2);
    double v = K(1,1)*y_distorted + K(1,2);
    
    if (u >= 0 && u < W && v >= 0 && v < H) {
      pts_2d.emplace_back(static_cast<float>(u), static_cast<float>(v));
      depths.push_back(pts_cam(idx, 2));
    }
  }

  if (pts_2d.empty()) {
    return img.clone();
  }

  // 归一化深度
  double max_depth = config.heatmap.max_depth;
  std::vector<double> norm_depths;
  for (double depth : depths) {
    norm_depths.push_back(std::min(depth / max_depth, 1.0));
  }

  // 生成颜色映射（简化版：红-蓝渐变）
  cv::Mat overlay = cv::Mat::zeros(H, W, CV_32FC3);
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    int u = static_cast<int>(pts_2d[i].x);
    int v = static_cast<int>(pts_2d[i].y);
    
    double d = norm_depths[i];
    cv::Vec3f color;
    color[0] = d;           // R: 近=红
    color[1] = 1.0 - d;     // G: 中=绿
    color[2] = 0.2;         // B: 远=蓝
    
    cv::circle(overlay, cv::Point(u, v), config.heatmap.point_size,
               cv::Scalar(color[0]*255, color[1]*255, color[2]*255), -1);
  }

  // 混合图像
  cv::addWeighted(result, 1.0 - config.heatmap.alpha, overlay,
                  config.heatmap.alpha, 0.0, result);
  
  cv::Mat result_8uc3;
  result.convertTo(result_8uc3, CV_8UC3);
  return result_8uc3;
}
#endif

void VisualizationV2::SaveErrorDistributionPlot(
    const std::vector<double>& errors,
    const std::string& title,
    const std::string& filename,
    double threshold) {
  
  if (errors.empty()) {
    LOG_WARNING("No errors to plot");
    return;
  }

  // 计算统计信息
  double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
  
  std::vector<double> sorted_errors = errors;
  std::sort(sorted_errors.begin(), sorted_errors.end());
  double median = sorted_errors[sorted_errors.size() / 2];
  
  double sum_sq = 0.0;
  for (double e : errors) {
    sum_sq += (e - mean) * (e - mean);
  }
  double stddev = std::sqrt(sum_sq / errors.size());
  
  double max_error = *std::max_element(errors.begin(), errors.end());
  double pct_within_1px = std::count_if(errors.begin(), errors.end(),
                                        [threshold](double e) { return e < threshold; }) * 100.0 / errors.size();

  // 生成Python脚本
  std::string script_path = output_dir_ + "/temp_plot_script.py";
  std::string output_path = output_dir_ + "/" + filename;
  
  std::ofstream script(script_path);
  script << "#!/usr/bin/env python3\n";
  script << "import matplotlib\n";
  script << "matplotlib.use('Agg')\n";
  script << "import matplotlib.pyplot as plt\n";
  script << "import numpy as np\n\n";
  
  script << "errors = [";
  for (size_t i = 0; i < errors.size(); ++i) {
    script << errors[i];
    if (i < errors.size() - 1) script << ", ";
  }
  script << "]\n\n";
  
  script << "fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))\n\n";
  
  // 直方图
  script << "ax1.hist(errors, bins=" << config_.error_distribution.bins 
          << ", color='steelblue', alpha=0.7, edgecolor='black')\n";
  script << "ax1.axvline(" << threshold << ", color='red', linestyle='--', "
          << "linewidth=2, label=f'Threshold (" << threshold << "px)')\n";
  script << "ax1.axvline(" << median << ", color='green', linestyle='--', "
          << "linewidth=2, label=f'Median (" << median << "px)')\n";
  script << "ax1.set_xlabel('Error (pixels)', fontsize=12)\n";
  script << "ax1.set_ylabel('Count', fontsize=12)\n";
  script << "ax1.set_title('Error Histogram', fontsize=13, fontweight='bold')\n";
  script << "ax1.legend()\n";
  script << "ax1.grid(alpha=0.3)\n\n";
  
  // CDF曲线
  script << "sorted_err = np.sort(errors)\n";
  script << "cdf = np.arange(1, len(sorted_err) + 1) / len(sorted_err)\n";
  script << "ax2.plot(sorted_err, cdf * 100, linewidth=2, color='steelblue')\n";
  script << "ax2.axhline(95, color='orange', linestyle='--', "
          << "linewidth=1.5, label='95th Percentile')\n";
  script << "ax2.axvline(" << threshold << ", color='red', linestyle='--', "
          << "linewidth=2, label=f'Threshold (" << threshold << "px)')\n";
  script << "ax2.set_xlabel('Error (pixels)', fontsize=12)\n";
  script << "ax2.set_ylabel('Cumulative Percentage (%)', fontsize=12)\n";
  script << "ax2.set_title('Cumulative Distribution', fontsize=13, fontweight='bold')\n";
  script << "ax2.legend()\n";
  script << "ax2.grid(alpha=0.3)\n";
  script << "ax2.set_ylim([0, 100])\n\n";
  
  // 统计信息文本框
  script << "stats_text = (\n";
  script << "    f\"Mean: {mean:.3f}px\\\\n\"\n";
  script << "    f\"Median: {median:.3f}px\\\\n\"\n";
  script << "    f\"Std: {stddev:.3f}px\\\\n\"\n";
  script << "    f\"Max: {max_error:.3f}px\\\\n\"\n";
  script << "    f\"<1px: {pct_within_1px:.1f}%\"\n";
  script << ")\n";
  script << "ax1.text(0.98, 0.98, stats_text, transform=ax1.transAxes,\n";
  script << "          fontsize=10, verticalalignment='top',\n";
  script << "          horizontalalignment='right',\n";
  script << "          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))\n\n";
  
  script << "plt.suptitle('" << title << "', fontsize=14, fontweight='bold', y=1.02)\n";
  script << "plt.tight_layout()\n";
  script << "plt.savefig('" << output_path << "', dpi=150, bbox_inches='tight')\n";
  script << "plt.close()\n";
  script.close();
  
  // 执行Python脚本
  std::string cmd = "python3 " + script_path + " && rm " + script_path;
  int ret = system(cmd.c_str());
  
  if (ret == 0) {
    LOG_INFO(std::string("Saved error distribution plot: ") + output_path);
  } else {
    LOG_WARNING("Failed to generate error distribution plot");
  }
}

#ifdef UNICALIB_USE_OPENCV
void VisualizationV2::SaveMultiFrameProjection(
    const std::vector<cv::Mat>& images,
    const std::vector<Eigen::MatrixXd>& pts_list,
    const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& extrinsics_list,
    const Eigen::Matrix3d& K,
    const Eigen::Vector4d& D,
    const std::vector<std::string>& labels,
    const std::string& filename) {
  
  if (images.empty()) {
    LOG_WARNING("No images to visualize");
    return;
  }

  int h = images[0].rows;
  int w = images[0].cols;
  int n_frames = std::min({static_cast<int>(images.size()),
                           static_cast<int>(pts_list.size()),
                           static_cast<int>(extrinsics_list.size())});
  
  int grid_cols = std::min(n_frames, 4);
  int grid_rows = (n_frames + grid_cols - 1) / grid_cols;
  
  cv::Mat result = cv::Mat::zeros(grid_rows * h, grid_cols * w, CV_8UC3);
  
  for (int i = 0; i < n_frames; ++i) {
    const auto& pts = pts_list[i];
    const auto& [R, t] = extrinsics_list[i];
    
    // 变换到相机坐标系
    Eigen::MatrixXd pts_cam = (R * pts.transpose() + t).transpose();
    
    // 生成热力图
    cv::Mat vis = DrawLidarHeatmap(images[i], pts_cam, K, D);
    
    // 添加标签
    std::string label = (i < static_cast<int>(labels.size())) ? labels[i] 
                                                         : "Frame " + std::to_string(i+1);
    cv::putText(vis, label, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
    
    // 计算统计信息
    int valid_pts = 0;
    for (int j = 0; j < pts_cam.rows(); ++j) {
      if (pts_cam(j, 2) > 0) valid_pts++;
    }
    std::string stats = "Points: " + std::to_string(valid_pts);
    cv::putText(vis, stats, cv::Point(10, h - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // 放入结果图
    int row = i / grid_cols;
    int col = i % grid_cols;
    cv::Rect roi(col*w, row*h, w, h);
    vis.copyTo(result(roi));
  }
  
  std::string output_path = output_dir_ + "/" + filename;
  cv::imwrite(output_path, result);
  LOG_INFO(std::string("Saved multi-frame comparison: ") + output_path);
}
#endif

void VisualizationV2::SaveSensorFrustum(
    const std::unordered_map<std::string, SensorConfig>& sensors,
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const std::vector<std::string>& camera_ids,
    const std::vector<std::string>& lidar_ids,
    const std::string& filename) {
  
  // 生成Python脚本来绘制3D图
  std::string script_path = output_dir_ + "/temp_frustum_script.py";
  std::string output_path = output_dir_ + "/" + filename;
  
  std::ofstream script(script_path);
  script << "#!/usr/bin/env python3\n";
  script << "import matplotlib\n";
  script << "matplotlib.use('Agg')\n";
  script << "import matplotlib.pyplot as plt\n";
  script << "from mpl_toolkits.mplot3d import Axes3D\n";
  script << "from mpl_toolkits.mplot3d.art3d import Poly3DCollection\n";
  script << "import numpy as np\n\n";
  
  script << "fig = plt.figure(figsize=(14, 12))\n";
  script << "ax = fig.add_subplot(111, projection='3d')\n\n";
  
  // 收集传感器位置
  std::vector<std::string> all_sensors;
  for (const auto& [name, _] : sensors) {
    all_sensors.push_back(name);
  }
  
  // 绘制相机视锥
  for (size_t i = 0; i < camera_ids.size(); ++i) {
    const std::string& cam_id = camera_ids[i];
    
    if (intrinsics.find(cam_id) == intrinsics.end() ||
        sensors.find(cam_id) == sensors.end()) {
      continue;
    }
    
    const auto& intr = intrinsics.at(cam_id);
    if (!intr.camera_pinhole) continue;
    
    const auto& cam = *intr.camera_pinhole;
    Eigen::Matrix3d K = cam.K;
    double fx = K(0, 0);
    double fy = K(1, 1);
    double cx = K(0, 2);
    double cy = K(1, 2);
    
    const auto& sensor = sensors.at(cam_id);
    int w = sensor.resolution ? sensor.resolution->first : 1920;
    int h = sensor.resolution ? sensor.resolution->second : 1080;
    
    // 查找外参 (格式: "lidar0-camera0")
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    for (const auto& [key, result] : extrinsics) {
      // 检查 key 是否包含当前相机ID
      if (key.find(cam_id) != std::string::npos && key.find("imu0") != std::string::npos) {
        R = result.rotation;
        t = result.translation;
        break;
      }
    }
    
    // 相机位置
    Eigen::Vector3d origin = -R.transpose() * t;
    
    script << "cam_origin = np.array([" << origin[0] << ", " << origin[1] 
            << ", " << origin[2] << "])\n";
    script << "ax.scatter(*cam_origin, c='blue', marker='s', s=200, label='"
            << cam_id << "')\n";
    
    // 视锥（简化版，仅绘制原点）
    script << "# Camera frustum: " << cam_id << "\n";
    script << "ax.quiver(" << origin[0] << ", " << origin[1] << ", " << origin[2]
            << ", 1.0, 0, 0, color='red', length=" << config_.sensor_frustum.axis_scale
            << ", arrow_length_ratio=0.2)\n";
    script << "ax.quiver(" << origin[0] << ", " << origin[1] << ", " << origin[2]
            << ", 0, 1.0, 0, color='green', length=" << config_.sensor_frustum.axis_scale
            << ", arrow_length_ratio=0.2)\n";
    script << "ax.quiver(" << origin[0] << ", " << origin[1] << ", " << origin[2]
            << ", 0, 0, 1.0, color='blue', length=" << config_.sensor_frustum.axis_scale
            << ", arrow_length_ratio=0.2)\n\n";
  }
  
  // 绘制LiDAR位置
  for (size_t i = 0; i < lidar_ids.size(); ++i) {
    const std::string& lidar_id = lidar_ids[i];
    
    if (extrinsics.find("imu0_" + lidar_id) == extrinsics.end()) {
      continue;
    }
    
    const auto& result = extrinsics.at("imu0_" + lidar_id);
    Eigen::Matrix3d R = result.rotation;
    Eigen::Vector3d t = result.translation;
    Eigen::Vector3d pos = -R.transpose() * t;
    
    script << "lidar_pos = np.array([" << pos[0] << ", " << pos[1] << ", " 
            << pos[2] << "])\n";
    script << "ax.scatter(*lidar_pos, c='red', marker='o', s=300, label='"
            << lidar_id << "')\n\n";
  }
  
  // IMU原点
  script << "ax.scatter(0, 0, 0, c='green', marker='D', s=300, label='IMU (Reference)')\n\n";
  
  // 设置坐标轴
  script << "ax.set_xlabel('X (m)', fontsize=12)\n";
  script << "ax.set_ylabel('Y (m)', fontsize=12)\n";
  script << "ax.set_zlabel('Z (m)', fontsize=12)\n";
  script << "ax.set_title('Sensor Frustums and Coordinate Systems', fontsize=14, fontweight='bold')\n";
  script << "ax.legend(loc='upper left', fontsize=10)\n";
  
  double max_range = 5.0;
  script << "ax.set_xlim([" << -max_range << ", " << max_range << "])\n";
  script << "ax.set_ylim([" << -max_range << ", " << max_range << "])\n";
  script << "ax.set_zlim([" << -max_range << ", " << max_range << "])\n";
  
  script << "plt.tight_layout()\n";
  script << "plt.savefig('" << output_path << "', dpi=150, bbox_inches='tight')\n";
  script << "plt.close()\n";
  script.close();
  
  // 执行Python脚本
  std::string cmd = "python3 " + script_path + " && rm " + script_path;
  int ret = system(cmd.c_str());
  
  if (ret == 0) {
    LOG_INFO(std::string("Saved sensor frustums: ") + output_path);
  } else {
    LOG_WARNING("Failed to generate sensor frustums plot");
  }
}

void VisualizationV2::SaveResidualPlot(
    const std::vector<double>& iteration_errors,
    const std::string& title,
    const std::string& filename) {
  
  if (iteration_errors.empty()) {
    LOG_WARNING("No iteration errors to plot");
    return;
  }
  
  std::string script_path = output_dir_ + "/temp_residual_script.py";
  std::string output_path = output_dir_ + "/" + filename;
  
  std::ofstream script(script_path);
  script << "#!/usr/bin/env python3\n";
  script << "import matplotlib\n";
  script << "matplotlib.use('Agg')\n";
  script << "import matplotlib.pyplot as plt\n";
  script << "import numpy as np\n\n";
  
  script << "iterations = np.arange(1, " << iteration_errors.size() + 1 << ")\n";
  script << "errors = [";
  for (size_t i = 0; i < iteration_errors.size(); ++i) {
    script << iteration_errors[i];
    if (i < iteration_errors.size() - 1) script << ", ";
  }
  script << "]\n\n";
  
  script << "fig, ax = plt.subplots(1, 1, figsize=(10, 6))\n";
  script << "ax.plot(iterations, errors, 'b-', linewidth=2, marker='o', markersize=4, "
          << "label='Residual')\n";
  script << "ax.set_xlabel('Iteration', fontsize=12)\n";
  script << "ax.set_ylabel('Residual', fontsize=12)\n";
  script << "ax.set_title('" << title << "', fontsize=14, fontweight='bold')\n";
  script << "ax.legend()\n";
  script << "ax.grid(True, alpha=0.3)\n\n";
  
  // 统计信息
  double improvement = (iteration_errors[0] - iteration_errors.back()) 
                       / iteration_errors[0] * 100.0;
  script << "stats_text = (\n";
  script << "    f\"Initial: {errors[0]:.4f}\\\\n\"\n";
  script << "    f\"Final: {errors[-1]:.4f}\\\\n\"\n";
  script << "    f\"Improvement: " << improvement << ":.1f}%\"\n";
  script << ")\n";
  script << "ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,\n";
  script << "          fontsize=10, verticalalignment='top',\n";
  script << "          bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))\n\n";
  
  script << "plt.tight_layout()\n";
  script << "plt.savefig('" << output_path << "', dpi=150, bbox_inches='tight')\n";
  script << "plt.close()\n";
  script.close();
  
  std::string cmd = "python3 " + script_path + " && rm " + script_path;
  int ret = system(cmd.c_str());
  
  if (ret == 0) {
    LOG_INFO(std::string("Saved residual convergence plot: ") + output_path);
  } else {
    LOG_WARNING("Failed to generate residual convergence plot");
  }
}

void VisualizationV2::SavePointcloudAlignment(
    const Eigen::MatrixXd& pts_source,
    const Eigen::MatrixXd& pts_target,
    const std::pair<Eigen::Matrix3d, Eigen::Vector3d>& transform,
    const std::string& filename) {
  
  std::string script_path = output_dir_ + "/temp_alignment_script.py";
  std::string output_path = output_dir_ + "/" + filename;
  
  std::ofstream script(script_path);
  script << "#!/usr/bin/env python3\n";
  script << "import matplotlib\n";
  script << "matplotlib.use('Agg')\n";
  script << "import matplotlib.pyplot as plt\n";
  script << "from mpl_toolkits.mplot3d import Axes3D\n";
  script << "import numpy as np\n\n";
  
  // 导入源点云
  script << "pts_source = np.array([\n";
  for (int i = 0; i < pts_source.rows(); ++i) {
    script << "  [" << pts_source(i, 0) << ", " << pts_source(i, 1) 
            << ", " << pts_source(i, 2) << "]";
    if (i < pts_source.rows() - 1) script << ",\n";
    else script << "\n";
  }
  script << "])\n\n";
  
  // 导入目标点云
  script << "pts_target = np.array([\n";
  for (int i = 0; i < pts_target.rows(); ++i) {
    script << "  [" << pts_target(i, 0) << ", " << pts_target(i, 1) 
            << ", " << pts_target(i, 2) << "]";
    if (i < pts_target.rows() - 1) script << ",\n";
    else script << "\n";
  }
  script << "])\n\n";
  
  // 变换矩阵
  const auto& [R, t] = transform;
  script << "R = np.array([\n";
  for (int i = 0; i < 3; ++i) {
    script << "  [";
    for (int j = 0; j < 3; ++j) {
      script << R(i, j);
      if (j < 2) script << ", ";
    }
    script << "]";
    if (i < 2) script << ",\n";
    else script << "\n";
  }
  script << "])\n\n";
  
  script << "t = np.array([" << t[0] << ", " << t[1] << ", " << t[2] << "])\n\n";
  
  // 变换后的点云
  script << "pts_transformed = (R @ pts_source.T + t.reshape(3, 1)).T\n\n";
  
  // 绘图
  script << "fig = plt.figure(figsize=(15, 6))\n\n";
  script << "# Before alignment\n";
  script << "ax1 = fig.add_subplot(121, projection='3d')\n";
  script << "ax1.scatter(pts_source[:,0], pts_source[:,1], pts_source[:,2], "
          << "c='blue', s=1, alpha=0.6, label='Source')\n";
  script << "ax1.scatter(pts_target[:,0], pts_target[:,1], pts_target[:,2], "
          << "c='red', s=1, alpha=0.6, label='Target')\n";
  script << "ax1.set_xlabel('X')\nax1.set_ylabel('Y')\nax1.set_zlabel('Z')\n";
  script << "ax1.set_title('Before Alignment', fontweight='bold')\n";
  script << "ax1.legend()\n\n";
  
  script << "# After alignment\n";
  script << "ax2 = fig.add_subplot(122, projection='3d')\n";
  script << "ax2.scatter(pts_transformed[:,0], pts_transformed[:,1], "
          << "pts_transformed[:,2], c='green', s=1, alpha=0.6, "
          << "label='Transformed Source')\n";
  script << "ax2.scatter(pts_target[:,0], pts_target[:,1], pts_target[:,2], "
          << "c='red', s=1, alpha=0.6, label='Target')\n";
  script << "ax2.set_xlabel('X')\nax2.set_ylabel('Y')\nax2.set_zlabel('Z')\n";
  script << "ax2.set_title('After Alignment', fontweight='bold')\n";
  script << "ax2.legend()\n\n";
  
  script << "plt.suptitle('Point Cloud Alignment Visualization', "
          << "fontsize=14, fontweight='bold')\n";
  script << "plt.tight_layout()\n";
  script << "plt.savefig('" << output_path << "', dpi=150, bbox_inches='tight')\n";
  script << "plt.close()\n";
  script.close();
  
  std::string cmd = "python3 " + script_path + " && rm " + script_path;
  int ret = system(cmd.c_str());
  
  if (ret == 0) {
    LOG_INFO(std::string("Saved pointcloud alignment: ") + output_path);
  } else {
    LOG_WARNING("Failed to generate pointcloud alignment plot");
  }
}

}  // namespace unicalib
