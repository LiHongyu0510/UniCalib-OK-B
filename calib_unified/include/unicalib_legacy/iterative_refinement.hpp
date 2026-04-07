/**
 * @file iterative_refinement.hpp
 * @brief CalibRefine 风格迭代精化模块
 * @reference CalibRefine (2025) - Deep Learning-Based Online Automatic Targetless LiDAR–Camera Calibration
 */
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <optional>
#include <string>

namespace unicalib {

// =============================================================================
// 旋转转换辅助函数声明
// =============================================================================

/**
 * @brief 旋转矩阵转旋转向量 (Rodrigues 公式)
 */
Eigen::Vector3d rotationMatrixToRotvec(const Eigen::Matrix3d& R);

/**
 * @brief 旋转向量转旋转矩阵 (Rodrigues 公式)
 */
Eigen::Matrix3d rotvecToRotationMatrix(const Eigen::Vector3d& rv);

/**
 * @brief 迭代精化配置
 */
struct RefinementConfig {
  bool enabled = true;
  int max_iterations = 3;
  double convergence_threshold = 0.001;   // 旋转变化阈值 (弧度)
  double translation_threshold = 0.001;   // 平移变化阈值 (米)
  bool early_stop = true;
  bool attention_refinement = false;      // ViT 注意力精化 (可选)
  double cost_tolerance = 1e-6;           // 代价函数容差
  
  static RefinementConfig Default() {
    return RefinementConfig{};
  }
  
  static RefinementConfig Aggressive() {
    RefinementConfig cfg;
    cfg.max_iterations = 5;
    cfg.convergence_threshold = 0.0005;
    cfg.translation_threshold = 0.0005;
    return cfg;
  }
  
  static RefinementConfig Fast() {
    RefinementConfig cfg;
    cfg.max_iterations = 2;
    cfg.convergence_threshold = 0.005;
    return cfg;
  }
};

/**
 * @brief 单次迭代结果
 */
struct IterationResult {
  int iteration;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double time_offset;
  double cost;
  double rotation_change;     // 相对于上次迭代 (弧度)
  double translation_change;  // 相对于上次迭代 (米)
  bool converged = false;
  
  std::string toString() const;
};

/**
 * @brief 多分辨率优化层级配置
 */
struct MultiResolutionLevel {
  int level;
  double knot_distance;   // B-spline 节点间距
  double cost;
  bool success;
};

/**
 * @brief 迭代精化器 - CalibRefine 风格
 * 
 * 核心能力:
 *   1. 迭代精化: 多轮优化逐渐收敛
 *   2. 注意力精化: Vision Transformer 处理非平面畸变
 *   3. 收敛检测: 自动停止已收敛的优化
 *   4. 历史追踪: 记录每次迭代的状态
 */
class IterativeRefiner {
 public:
  using CostFunction = std::function<double(
      const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t,
      double td)>;
  
  using JacobianFunction = std::function<void(
      const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t,
      double td,
      Eigen::VectorXd& grad)>;
  
  explicit IterativeRefiner(const RefinementConfig& config = RefinementConfig::Default());
  
  /**
   * @brief 迭代精化外参
   * @param initial_R 初始旋转矩阵
   * @param initial_t 初始平移向量
   * @param initial_td 初始时间偏移
   * @param cost_function 代价函数
   * @param jacobian 可选的雅可比矩阵函数
   * @return (优化后的R, 优化后的t, 优化后的td, 迭代历史)
   */
  std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, std::vector<IterationResult>>
  refine(
      const Eigen::Matrix3d& initial_R,
      const Eigen::Vector3d& initial_t,
      double initial_td,
      CostFunction cost_function,
      std::optional<JacobianFunction> jacobian = std::nullopt);
  
  /**
   * @brief 获取迭代历史
   */
  const std::vector<IterationResult>& history() const { return history_; }
  
  /**
   * @brief 检查是否收敛
   */
  bool isConverged() const { return !history_.empty() && history_.back().converged; }
  
  /**
   * @brief 计算两个旋转矩阵之间的角度距离 (弧度)
   */
  static double rotationDistance(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2);

 private:
  RefinementConfig config_;
  std::vector<IterationResult> history_;
  
  /**
   * @brief 执行单次优化迭代
   */
  std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, double>
  singleIteration(
      const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t,
      double td,
      CostFunction cost_function,
      std::optional<JacobianFunction> jacobian);
  
  /**
   * @brief 检查是否收敛
   */
  bool checkConvergence(double R_change, double t_change) const;
};

/**
 * @brief 注意力精化器 (Vision Transformer 风格)
 * 
 * 基于 CalibRefine 的 attention-based post-refinement
 * 处理非平面畸变和复杂场景
 */
class AttentionRefiner {
 public:
  explicit AttentionRefiner(bool use_vit = false);
  
  /**
   * @brief 计算注意力权重
   * @param image 输入图像
   * @param point_projections 投影点 Nx2
   * @return 权重向量 N
   */
  Eigen::VectorXd computeAttentionWeights(
      const Eigen::MatrixXd& image,
      const Eigen::Matrix<double, -1, 2>& point_projections);
  
  /**
   * @brief 基于边缘的权重计算 (ViT 不可用时的替代方案)
   */
  Eigen::VectorXd computeEdgeBasedWeights(
      const Eigen::MatrixXd& edges,
      const Eigen::Matrix<double, -1, 2>& points);

 private:
  bool use_vit_;
};

/**
 * @brief 多分辨率优化器
 * 
 * 在不同分辨率层级上优化，逐步精细化
 */
class MultiResolutionOptimizer {
 public:
  MultiResolutionOptimizer(int levels = 3, 
                           const std::vector<double>& knot_factors = {4.0, 2.0, 1.0});
  
  /**
   * @brief 多分辨率优化
   * @param initial_params 初始参数
   * @param cost_function_factory 工厂函数，接收 knot_distance 返回代价函数
   * @param base_knot_distance 基准节点间距
   * @return (优化参数, 各层级结果)
   */
  std::pair<Eigen::VectorXd, std::vector<MultiResolutionLevel>>
  optimizeMultiresolution(
      const Eigen::VectorXd& initial_params,
      std::function<IterativeRefiner::CostFunction(double)> cost_function_factory,
      double base_knot_distance = 0.02);

 private:
  int levels_;
  std::vector<double> knot_factors_;
};

/**
 * @brief 闭环一致性检查器
 * 
 * 验证多传感器标定的闭环一致性
 * 例如: IMU-LiDAR + LiDAR-Camera + Camera-IMU 应形成闭环
 */
class LoopClosureValidator {
 public:
  /**
   * @brief 检查闭环一致性
   * @param extrinsics 外参字典 (pair_key -> CalibResult)
   * @param threshold 一致性阈值
   * @return (是否通过, 误差)
   */
  std::pair<bool, double> validateLoopClosure(
      const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics,
      double threshold = 0.01);
  
  /**
   * @brief 计算三元组闭环误差
   * T_AB * T_BC * T_CA 应该 ≈ I
   */
  static double computeLoopError(
      const Eigen::Matrix4d& T_AB,
      const Eigen::Matrix4d& T_BC,
      const Eigen::Matrix4d& T_CA);
};

}  // namespace unicalib
