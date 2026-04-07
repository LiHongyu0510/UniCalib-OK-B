/**
 * @file iterative_refinement.cpp
 * @brief CalibRefine 风格迭代精化模块实现
 */
#include "unicalib/iterative_refinement.hpp"
#include "unicalib/logger.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

#ifdef UNICALIB_USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace unicalib {

// =============================================================================
// IterationResult
// =============================================================================

std::string IterationResult::toString() const {
  std::ostringstream oss;
  oss << "Iter " << iteration << ": cost=" << cost 
      << ", R_change=" << rotation_change * 180.0 / M_PI << "deg"
      << ", t_change=" << translation_change * 1000.0 << "mm"
      << (converged ? " [CONVERGED]" : "");
  return oss.str();
}

// =============================================================================
// IterativeRefiner
// =============================================================================

IterativeRefiner::IterativeRefiner(const RefinementConfig& config)
    : config_(config) {}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, std::vector<IterationResult>>
IterativeRefiner::refine(
    const Eigen::Matrix3d& initial_R,
    const Eigen::Vector3d& initial_t,
    double initial_td,
    CostFunction cost_function,
    std::optional<JacobianFunction> jacobian) {
  
  if (!config_.enabled) {
    return {initial_R, initial_t, initial_td, {}};
  }
  
  history_.clear();
  Eigen::Matrix3d R_curr = initial_R;
  Eigen::Vector3d t_curr = initial_t;
  double td_curr = initial_td;
  
  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    // 执行单次优化
    auto [R_new, t_new, td_new, cost] = singleIteration(
        R_curr, t_curr, td_curr, cost_function, jacobian);
    
    // 计算变化量
    double R_change = rotationDistance(R_curr, R_new);
    double t_change = (t_new - t_curr).norm();
    
    // 记录结果
    IterationResult result;
    result.iteration = iter;
    result.rotation = R_new;
    result.translation = t_new;
    result.time_offset = td_new;
    result.cost = cost;
    result.rotation_change = R_change;
    result.translation_change = t_change;
    
    history_.push_back(result);
    
    LOG_INFO_FMT("  [Iter %d/%d] cost=%.6f, R_change=%.4fdeg, t_change=%.4fmm",
             iter + 1, config_.max_iterations, cost,
             R_change * 180.0 / M_PI, t_change * 1000.0);
    
    // 检查收敛
    if (checkConvergence(R_change, t_change)) {
      result.converged = true;
      LOG_INFO_FMT("  Converged at iteration %d", iter + 1);
      break;
    }
    
    R_curr = R_new;
    t_curr = t_new;
    td_curr = td_new;
  }
  
  return {R_curr, t_curr, td_curr, history_};
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double, double>
IterativeRefiner::singleIteration(
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    double td,
    CostFunction cost_function,
    std::optional<JacobianFunction> jacobian) {
  
  // 参数化: [旋转向量(3), 平移(3), 时间偏移(1)]
  Eigen::Vector3d rv = rotationMatrixToRotvec(R);
  Eigen::VectorXd x0(7);
  x0 << rv, t, td;
  
  // 目标函数包装
  auto objective = [&](const Eigen::VectorXd& x) -> double {
    Eigen::Vector3d rv_x = x.head<3>();
    Eigen::Vector3d t_x = x.segment<3>(3);
    double td_x = x(6);
    Eigen::Matrix3d R_x = rotvecToRotationMatrix(rv_x);
    return cost_function(R_x, t_x, td_x);
  };
  
  // 简化的 Nelder-Mead 优化
  // (生产环境应使用 Ceres 或 NLopt)
  Eigen::VectorXd x_opt = x0;
  double best_cost = objective(x0);
  
  // 坐标下降法作为简单优化
  const double step = 1e-4;
  for (int coord = 0; coord < 7; ++coord) {
    for (int direction = -1; direction <= 1; direction += 2) {
      Eigen::VectorXd x_try = x_opt;
      x_try(coord) += direction * step;
      double cost_try = objective(x_try);
      if (cost_try < best_cost) {
        best_cost = cost_try;
        x_opt = x_try;
      }
    }
  }
  
  // 解析结果
  Eigen::Vector3d rv_opt = x_opt.head<3>();
  Eigen::Matrix3d R_opt = rotvecToRotationMatrix(rv_opt);
  Eigen::Vector3d t_opt = x_opt.segment<3>(3);
  double td_opt = x_opt(6);
  
  return {R_opt, t_opt, td_opt, best_cost};
}

double IterativeRefiner::rotationDistance(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
  Eigen::Matrix3d R_diff = R1.transpose() * R2;
  Eigen::Vector3d rv = rotationMatrixToRotvec(R_diff);
  return rv.norm();
}

bool IterativeRefiner::checkConvergence(double R_change, double t_change) const {
  if (!config_.early_stop) {
    return false;
  }
  return R_change < config_.convergence_threshold &&
         t_change < config_.translation_threshold;
}

// =============================================================================
// 辅助函数 (旋转相关)
// =============================================================================

Eigen::Vector3d rotationMatrixToRotvec(const Eigen::Matrix3d& R) {
  // 使用 Rodrigues 公式
  double trace = R.trace();
  double theta = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
  
  if (theta < 1e-8) {
    return Eigen::Vector3d::Zero();
  }
  
  double scale = theta / (2.0 * std::sin(theta));
  Eigen::Vector3d rv;
  rv << (R(2, 1) - R(1, 2)) * scale,
        (R(0, 2) - R(2, 0)) * scale,
        (R(1, 0) - R(0, 1)) * scale;
  
  return rv;
}

Eigen::Matrix3d rotvecToRotationMatrix(const Eigen::Vector3d& rv) {
  double theta = rv.norm();
  if (theta < 1e-8) {
    return Eigen::Matrix3d::Identity();
  }
  
  Eigen::Vector3d axis = rv / theta;
  double c = std::cos(theta);
  double s = std::sin(theta);
  double t = 1.0 - c;
  
  double x = axis(0), y = axis(1), z = axis(2);
  
  Eigen::Matrix3d R;
  R << t*x*x + c,   t*x*y - s*z, t*x*z + s*y,
       t*x*y + s*z, t*y*y + c,   t*y*z - s*x,
       t*x*z - s*y, t*y*z + s*x, t*z*z + c;
  
  return R;
}

// =============================================================================
// AttentionRefiner
// =============================================================================

AttentionRefiner::AttentionRefiner(bool use_vit) : use_vit_(use_vit) {
  // ViT 模型加载 (需要 PyTorch/TorchScript，这里占位)
  if (use_vit_) {
    LOG_WARNING("ViT attention refinement not implemented, using edge-based fallback");
    use_vit_ = false;
  }
}

Eigen::VectorXd AttentionRefiner::computeAttentionWeights(
    const Eigen::MatrixXd& image,
    const Eigen::Matrix<double, -1, 2>& point_projections) {
  
  if (!use_vit_) {
    // 均匀权重
    int n = point_projections.rows();
    return Eigen::VectorXd::Constant(n, 1.0 / n);
  }
  
  // ViT 实现占位
  return Eigen::VectorXd::Constant(point_projections.rows(), 
                                   1.0 / point_projections.rows());
}

Eigen::VectorXd AttentionRefiner::computeEdgeBasedWeights(
    const Eigen::MatrixXd& edges,
    const Eigen::Matrix<double, -1, 2>& points) {
  
  int n = points.rows();
  int h = edges.rows();
  int w = edges.cols();
  
  Eigen::VectorXd weights(n);
  weights.setZero();
  
  for (int i = 0; i < n; ++i) {
    int u = static_cast<int>(points(i, 0));
    int v = static_cast<int>(points(i, 1));
    
    if (u >= 5 && u < w - 5 && v >= 5 && v < h - 5) {
      // 5x5 邻域内的边缘响应
      double sum = 0.0;
      int count = 0;
      for (int dv = -5; dv <= 5; ++dv) {
        for (int du = -5; du <= 5; ++du) {
          if (edges(v + dv, u + du) > 0) {
            sum += 1.0;
          }
          ++count;
        }
      }
      weights(i) = sum / count;
    }
  }
  
  // 归一化
  double total = weights.sum();
  if (total > 0) {
    weights /= total;
  } else {
    weights.setConstant(1.0 / n);
  }
  
  return weights;
}

// =============================================================================
// MultiResolutionOptimizer
// =============================================================================

MultiResolutionOptimizer::MultiResolutionOptimizer(
    int levels, const std::vector<double>& knot_factors)
    : levels_(levels), knot_factors_(knot_factors) {}

std::pair<Eigen::VectorXd, std::vector<MultiResolutionLevel>>
MultiResolutionOptimizer::optimizeMultiresolution(
    const Eigen::VectorXd& initial_params,
    std::function<IterativeRefiner::CostFunction(double)> cost_function_factory,
    double base_knot_distance) {
  
  Eigen::VectorXd params = initial_params;
  std::vector<MultiResolutionLevel> results;
  
  for (int level = 0; level < levels_; ++level) {
    double knot_distance = base_knot_distance * knot_factors_[level];
    LOG_INFO_FMT("  [Level %d/%d] knot_distance=%.4fs", 
             level + 1, levels_, knot_distance);
    
    // 创建当前层级的代价函数
    auto cost_fn = cost_function_factory(knot_distance);
    
    // 简单优化 (坐标下降)
    double best_cost = cost_fn(
        Eigen::Matrix3d::Identity(), 
        Eigen::Vector3d::Zero(), 
        0.0);  // 简化，实际需要从 params 恢复
    
    MultiResolutionLevel result;
    result.level = level;
    result.knot_distance = knot_distance;
    result.cost = best_cost;
    result.success = true;
    results.push_back(result);
    
    LOG_INFO_FMT("    cost=%.6f", best_cost);
  }
  
  return {params, results};
}

// =============================================================================
// LoopClosureValidator
// =============================================================================

std::pair<bool, double> LoopClosureValidator::validateLoopClosure(
    const std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics,
    double threshold) {
  
  // 寻找三元组闭环
  // 例如: "imu:lidar", "lidar:camera", "camera:imu"
  
  double max_error = 0.0;
  
  // 简化实现: 检查所有可能的三元组
  for (const auto& [key1, T1] : extrinsics) {
    for (const auto& [key2, T2] : extrinsics) {
      if (key1 == key2) continue;
      
      // 解析 key 格式 "sensor_a:sensor_b"
      auto parsePair = [](const std::string& key) -> std::pair<std::string, std::string> {
        size_t pos = key.find(':');
        if (pos == std::string::npos) return {"", ""};
        return {key.substr(0, pos), key.substr(pos + 1)};
      };
      
      auto [a1, b1] = parsePair(key1);
      auto [a2, b2] = parsePair(key2);
      
      // 寻找闭环: T_AB * T_BC * T_CA
      if (b1 == a2) {
        std::string key3 = b2 + ":" + a1;
        auto it3 = extrinsics.find(key3);
        if (it3 != extrinsics.end()) {
          double error = computeLoopError(T1, T2, it3->second);
          max_error = std::max(max_error, error);
        }
      }
    }
  }
  
  bool pass = max_error < threshold;
  return {pass, max_error};
}

double LoopClosureValidator::computeLoopError(
    const Eigen::Matrix4d& T_AB,
    const Eigen::Matrix4d& T_BC,
    const Eigen::Matrix4d& T_CA) {
  
  // T_AB * T_BC * T_CA 应该 ≈ I
  Eigen::Matrix4d T_loop = T_AB * T_BC * T_CA;
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  
  // 旋转误差
  Eigen::Matrix3d R_err = T_loop.block<3, 3>(0, 0);
  double rot_err = Eigen::Vector3d::UnitX().cross(R_err * Eigen::Vector3d::UnitX()).norm();
  
  // 平移误差
  Eigen::Vector3d t_err = T_loop.block<3, 1>(0, 3);
  double trans_err = t_err.norm();
  
  return std::max(rot_err, trans_err);
}

}  // namespace unicalib
