/**
 * UniCalib C++ — 增强版可视化工具 V2
 * 对应 Python: unicalib/utils/visualization_v2.py
 * 
 * 功能:
 * - 热力图投影 (LiDAR点云到图像)
 * - 误差分布图 (直方图 + CDF)
 * - 传感器视锥图 (3D坐标系和相机视锥)
 * - 多帧对比图
 * - 残差收敛曲线
 * - 点云对齐可视化
 */
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

#ifdef UNICALIB_USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

#include "calib_result.hpp"
#include "sensor_config.hpp"

namespace unicalib {

/** 可视化配置 */
struct VisualizationConfig {
  // 热力图配置
  struct {
    double max_depth = 50.0;      // 最大深度（米）
    int point_size = 3;            // 点大小
    double alpha = 0.7;            // 透明度 [0,1]
    int colormap = 0;              // OpenCV颜色映射 (COLORMAP_JET=4)
  } heatmap;

  // 误差分布图配置
  struct {
    double threshold = 1.0;        // 合格阈值（像素）
    int bins = 50;                 // 直方图bin数量
  } error_distribution;

  // 传感器视锥图配置
  struct {
    double frustum_depth = 5.0;    // 视锥深度（米）
    double axis_scale = 1.0;       // 坐标轴长度
  } sensor_frustum;
};

/** 增强版可视化工具 */
class VisualizationV2 {
 public:
  explicit VisualizationV2(const std::string& output_dir);
  ~VisualizationV2() = default;

  /**
   * 将LiDAR点云投影到图像生成热力图
   * @param img 原始图像 (CV_8UC3)
   * @param pts_cam 相机坐标系下的点云 (N×3)
   * @param K 相机内参矩阵 (3×3)
   * @param D 畸变系数 (4×1)
   * @param config 可视化配置
   * @return 叠加了热力图的图像
   */
#ifdef UNICALIB_USE_OPENCV
  cv::Mat DrawLidarHeatmap(
    const cv::Mat& img,
    const Eigen::MatrixXd& pts_cam,
    const Eigen::Matrix3d& K,
    const Eigen::Vector4d& D,
    const VisualizationConfig& config = VisualizationConfig());
#endif

  /**
   * 保存误差分布图 (直方图 + CDF)
   * @param errors 误差值数组
   * @param title 图表标题
   * @param filename 输出文件名
   * @param threshold 合格阈值
   */
  void SaveErrorDistributionPlot(
    const std::vector<double>& errors,
    const std::string& title,
    const std::string& filename,
    double threshold = 1.0);

  /**
   * 保存多帧投影对比图
   * @param images 图像列表
   * @param pts_list 对应的点云列表（世界坐标系）
   * @param extrinsics_list 外参列表
   * @param K 相机内参
   * @param D 畸变系数
   * @param labels 每个子图的标签
   * @param filename 输出文件名
   */
#ifdef UNICALIB_USE_OPENCV
  void SaveMultiFrameProjection(
    const std::vector<cv::Mat>& images,
    const std::vector<Eigen::MatrixXd>& pts_list,
    const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& extrinsics_list,
    const Eigen::Matrix3d& K,
    const Eigen::Vector4d& D,
    const std::vector<std::string>& labels,
    const std::string& filename);
#endif

  /**
   * 保存传感器视锥和坐标系3D概览图
   * @param sensors 传感器配置字典
   * @param intrinsics 内参结果字典
   * @param extrinsics 外参结果字典
   * @param camera_ids 要显示的相机ID列表
   * @param lidar_ids 要显示的LiDAR ID列表
   * @param filename 输出文件名
   */
  void SaveSensorFrustum(
    const std::unordered_map<std::string, SensorConfig>& sensors,
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const std::vector<std::string>& camera_ids,
    const std::vector<std::string>& lidar_ids,
    const std::string& filename);

  /**
   * 保存优化残差收敛曲线
   * @param iteration_errors 每次迭代的误差列表
   * @param title 图表标题
   * @param filename 输出文件名
   */
  void SaveResidualPlot(
    const std::vector<double>& iteration_errors,
    const std::string& title,
    const std::string& filename);

  /**
   * 保存点云对齐可视化（变换前后对比）
   * @param pts_source 源点云 (N×3)
   * @param pts_target 目标点云 (M×3)
   * @param transform 变换矩阵 (R, t)
   * @param filename 输出文件名
   */
  void SavePointcloudAlignment(
    const Eigen::MatrixXd& pts_source,
    const Eigen::MatrixXd& pts_target,
    const std::pair<Eigen::Matrix3d, Eigen::Vector3d>& transform,
    const std::string& filename);

  void SetConfig(const VisualizationConfig& config) { config_ = config; }

 private:
  std::string output_dir_;
  VisualizationConfig config_;

  /**
   * 生成Python脚本来创建图表（使用matplotlib）
   */
  void GenerateMatplotlibScript(
    const std::string& script_filename,
    const std::vector<double>& errors,
    const std::string& title,
    double threshold);
};

}  // namespace unicalib
