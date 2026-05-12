/**
 * UniCalib — 数学安全工具库
 * 
 * 目的：提供统一的数值稳定性检查和除零保护
 * 
 * 功能：
 * - 数值有效性检查（NaN, Inf, Finite）
 * - 除零和接近零保护
 * - 归一化安全性（向量、四元数）
 * - 阈值常量定义
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace ns_unicalib {
namespace MathSafety {

// ===================================================================
// 数值精度常量
// ===================================================================

constexpr double EPS_DEFAULT = 1e-10;      // 默认机器精度
constexpr double EPS_ROTATION = 1e-8;     // 旋转相关精度
constexpr double EPS_QUATERNION = 1e-10; // 四元数精度
constexpr double EPS_DEPTH = 1e-6;        // 深度精度
constexpr double EPS_SVD = 1e-12;         // SVD 奇异值精度
constexpr double EPS_NORM = 1e-8;         // 归一化零向量精度

// 角度转换常量
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// ===================================================================
// 数值有效性检查模板
// ===================================================================

/**
 * @brief 检查单个值是否有效（非 NaN, 非 Inf）
 */
template<typename T>
inline bool isValid(const T& val) {
    return !std::isnan(val) && !std::isinf(val);
}

/**
 * @brief 检查单个值是否有限（非 NaN, 非 Inf, 有限）
 */
template<typename T>
inline bool isFinite(const T& val) {
    return std::isfinite(val);
}

/**
 * @brief 检查单个值是否为正
 */
template<typename T>
inline bool isPositive(const T& val) {
    return val > T(0);
}

/**
 * @brief 检查向量是否所有元素有效
 */
template<typename Derived>
inline bool isValid(const Eigen::MatrixBase<Derived>& mat) {
    return mat.array().isFinite().all();
}

/**
 * @brief 检查向量是否所有元素有限
 */
template<typename Derived>
inline bool isFinite(const Eigen::MatrixBase<Derived>& mat) {
    return mat.array().isFinite().all();
}

/**
 * @brief 检查矩阵是否接近奇异（行列式接近零）
 */
template<typename Derived, int Rows, int Cols>
inline bool isNearSingular(const Eigen::Matrix<Derived, Rows, Cols>& mat,
                         double eps = EPS_DEFAULT) {
    if (mat.rows() != mat.cols()) return false;
    return std::abs(mat.determinant()) < eps;
}

/**
 * @brief 检查向量是否接近零向量
 */
template<typename Derived>
inline bool isNearZero(const Eigen::MatrixBase<Derived>& vec,
                    double eps = EPS_ROTATION) {
    return vec.norm() < eps;
}

// ===================================================================
// 除零保护模板
// ===================================================================

/**
 * @brief 安全除法：检查分母是否接近零
 * @throws std::runtime_error 如果分母接近零
 */
template<typename T>
inline T safeDivide(const T& numerator, const T& denominator,
                   const T& epsilon = EPS_DEFAULT) {
    if (std::abs(denominator) < epsilon) {
        throw std::runtime_error(
            "[MathSafety] Division by zero or near-zero: numerator=" +
            std::to_string(numerator) + ", denominator=" +
            std::to_string(denominator) + ", eps=" + std::to_string(epsilon));
        }
    return numerator / denominator;
}

/**
 * @brief 安全除法（向量版）：逐元素除法
 */
template<typename Derived, typename OtherDerived>
inline auto safeDivide(const Eigen::MatrixBase<Derived>& numerator,
                     const Eigen::MatrixBase<OtherDerived>& denominator,
                     double epsilon = EPS_DEFAULT)
    -> decltype((numerator.derived() / denominator.derived().array()).eval()) {
    
    auto denom_array = denominator.derived().array();
    auto denom_sq = denom_array.square();
    
    // 检查每个元素是否接近零
    auto mask = (denom_sq < epsilon * epsilon).template cast<double>();
    
    if (mask.any()) {
        // 如果有任何分母接近零，抛出异常
        throw std::runtime_error("[MathSafety] Division by near-zero in vector operation");
    }
    
    return (numerator.derived() / denominator.derived().array()).eval();
}

/**
 * @brief 安全除法（标量版）：返回安全值或默认值
 * @return 结果，如果分母接近零则返回 fallback
 */
template<typename T>
inline T safeDivideWithFallback(const T& numerator, const T& denominator,
                                const T& epsilon = EPS_DEFAULT,
                                const T& fallback = T(0)) {
    if (std::abs(denominator) < epsilon) {
        return fallback;
    }
    return numerator / denominator;
}

// ===================================================================
// 归一化安全性
// ===================================================================

/**
 * @brief 安全的向量归一化：检查是否接近零
 * @return 归一化后的向量，如果接近零则返回零向量
 */
template<typename Derived>
inline auto safeNormalize(const Eigen::MatrixBase<Derived>& vec,
                           double eps = EPS_NORM)
    -> typename Derived::PlainObject {
    
    using Scalar = typename Derived::Scalar;
    double norm = vec.norm();
    if (norm < eps) {
        // 向量接近零，返回零向量
        return Derived::PlainObject::Zero(vec.size());
    }
    return vec.derived() / static_cast<Scalar>(norm);
}

/**
 * @brief 安全的四元数归一化：检查模长
 * @return 归一化后的四元数
 */
inline Eigen::Quaterniond safeNormalize(const Eigen::Quaterniond& q,
                                     double eps = EPS_QUATERNION) {
    double q_norm = q.norm();
    if (q_norm < eps) {
        // 四元数接近零，返回单位四元数
        return Eigen::Quaterniond::Identity();
    }
    return q.normalized();
}

/**
 * @brief 安全的归一化（返回归一化状态）
 * @return 归一化后的向量和状态（是否成功）
 */
template<typename Derived>
inline typename Derived::PlainObject safeNormalizeWithStatus(
    const Eigen::MatrixBase<Derived>& vec,
    bool& success,
    double eps = EPS_NORM) {
    
    using Scalar = typename Derived::Scalar;
    double norm = vec.norm();
    success = (norm >= eps);
    
    if (success) {
        return vec.derived() / static_cast<Scalar>(norm);
    } else {
        // 向量接近零，返回零向量
        return Derived::PlainObject::Zero(vec.size());
    }
}

// ===================================================================
// 数值稳定性检查
// ===================================================================

/**
 * @brief 检查并限制值在指定范围内
 */
template<typename T>
inline T clamp(const T& value, const T& min_val, const T& max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief 检查对数运算的安全性
 * @return 是否安全（参数 > 0）
 */
template<typename T>
inline bool isSafeForLog(const T& value) {
    return value > T(0);
}

/**
 * @brief 安全的对数运算
 * @return 结果，或 NaN（如果输入 <= 0）
 */
template<typename T>
inline T safeLog(const T& value) {
    if (value <= T(0)) {
        return std::numeric_limits<T>::quiet_NaN();
    }
    return std::log(value);
}

/**
 * @brief 安全的 sqrt 运算
 * @return 结果，或零（如果输入 < 0）
 */
template<typename T>
inline T safeSqrt(const T& value) {
    if (value < T(0)) {
        return T(0);
    }
    return std::sqrt(value);
}

/**
 * @brief 安全的 acos 运算
 * @return 结果，或零（如果输入 > 1 或 < -1）
 */
template<typename T>
inline T safeAcos(const T& value) {
    T abs_val = value;
    if (abs_val < T(-1)) abs_val = T(-1);
    if (abs_val > T(1)) abs_val = T(1);
    
    return std::acos(abs_val);
}

/**
 * @brief 检查角度是否在有效范围内
 */
template<typename T>
inline bool isValidAngle(const T& angle_rad) {
    return !std::isnan(angle_rad) && std::abs(angle_rad) <= M_PI;
}

// ===================================================================
// 辅助函数
// ===================================================================

/**
 * @brief 将角度从度转换为弧度
 */
inline double degToRad(double deg) {
    return deg * DEG_TO_RAD;
}

/**
 * @brief 将角度从弧度转换为度
 */
inline double radToDeg(double rad) {
    return rad * RAD_TO_DEG;
}

/**
 * @brief 检查时间戳有效性
 * @return 是否有效（非 NaN, 非 Inf, 正数）
 */
inline bool isValidTimestamp(double timestamp) {
    return isValid(timestamp) && isPositive(timestamp);
}

/**
 * @brief 检查时间间隔的有效性
 * @return 是否有效（非 NaN, 非 Inf, 合理范围）
 */
inline bool isValidTimeDelta(double dt, double max_dt = 10.0) {
    return isValid(dt) && std::abs(dt) <= max_dt;
}

/**
 * @brief 规范化时间戳到 [0, 1] 范围
 * @param timestamp 要规范化的时间戳
 * @param min_time 最小时间戳
 * @param max_time 最大时间戳
 * @return 规范化后的时间戳
 */
inline double normalizeTimestamp(double timestamp, double min_time, double max_time) {
    double range = max_time - min_time;
    if (range < 1e-10) {
        return 0.5; // 时间范围太小
    }
    return (timestamp - min_time) / range;
}

// ===================================================================
// 统计学辅助函数
// ===================================================================

/**
 * @brief 计算鲁棒的均值（中位数）
 * @param data 数据向量
 * @return 中位数
 */
template<typename T>
inline T robustMedian(const std::vector<T>& data) {
    if (data.empty()) return T(0);
    
    std::vector<T> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    
    size_t n = sorted_data.size();
    if (n % 2 == 0) {
        return (sorted_data[n/2 - 1] + sorted_data[n/2]) / T(2);
    } else {
        return sorted_data[n/2];
    }
}

/**
 * @brief 计算鲁棒的方差（MAD - Median Absolute Deviation）
 * @param data 数据向量
 * @param median 中位数（可选，如果不提供则内部计算）
 * @return MAD
 */
template<typename T>
inline T robustMAD(const std::vector<T>& data, const T* median_ptr = nullptr) {
    if (data.empty()) return T(0);
    
    T median = median_ptr ? *median_ptr : robustMedian(data);
    
    std::vector<T> abs_deviations;
    abs_deviations.reserve(data.size());
    for (const auto& val : data) {
        abs_deviations.push_back(std::abs(val - median));
    }
    
    return robustMedian(abs_deviations);
}

/**
 * @brief 计算百分位数
 * @param data 数据向量
 * @param percentile 百分位数 (0-100)
 * @return 百分位数值
 */
template<typename T>
inline T percentile(const std::vector<T>& data, double percentile) {
    if (data.empty()) return T(0);
    
    std::vector<T> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    
    double idx = percentile / 100.0 * (sorted_data.size() - 1);
    size_t i0 = static_cast<size_t>(std::floor(idx));
    size_t i1 = static_cast<size_t>(std::ceil(idx));
    
    if (i0 >= sorted_data.size() - 1) return sorted_data.back();
    if (i1 >= sorted_data.size() - 1) return sorted_data.back();
    
    return (sorted_data[i0] * (1 - (idx - i0)) + 
            sorted_data[i1] * (idx - i0)) / T(2);
}

/**
 * @brief 移除离群值（使用 IQR 方法）
 * @param data 数据向量
 * @param multiplier IQR 倍数（默认 1.5）
 * @return 过滤后的数据
 */
template<typename T>
inline std::vector<T> removeOutliersIQR(
    const std::vector<T>& data, 
    double multiplier = 1.5) {
    
    if (data.size() < 4) return data;
    
    T q1 = percentile(data, 25);
    T q3 = percentile(data, 75);
    T iqr = q3 - q1;
    
    T lower = q1 - multiplier * iqr;
    T upper = q3 + multiplier * iqr;
    
    std::vector<T> filtered;
    filtered.reserve(data.size());
    
    for (const auto& val : data) {
        if (val >= lower && val <= upper) {
            filtered.push_back(val);
        }
    }
    
    return filtered;
}

// ===================================================================
// 调试和日志辅助
// ===================================================================

/**
 * @brief 数值信息的字符串表示
 */
template<typename T>
inline std::string toString(const T& value) {
    return std::to_string(value);
}

/**
 * @brief 向量信息的字符串表示
 */
template<typename Derived>
inline std::string toString(const Eigen::MatrixBase<Derived>& vec, int max_elements = 10) {
    std::ostringstream oss;
    oss << "[";
    int n = std::min(max_elements, static_cast<int>(vec.size()));
    for (int i = 0; i < n; ++i) {
        oss << vec(i);
        if (i < n - 1) oss << ", ";
    }
    oss << "]";
    if (vec.size() > max_elements) oss << " ...";
    return oss.str();
}

/**
 * @brief 矩阵信息的字符串表示
 */
template<typename Derived>
inline std::string toString(const Eigen::MatrixBase<Derived>& mat, int max_rows = 3, int max_cols = 3) {
    std::ostringstream oss;
    int rows = std::min(max_rows, mat.rows());
    int cols = std::min(max_cols, mat.cols());
    
    oss << "[";
    for (int i = 0; i < rows; ++i) {
        if (i > 0) oss << ";\n ";
        for (int j = 0; j < cols; ++j) {
            oss << mat(i, j);
            if (j < cols - 1) oss << ", ";
        }
    }
    oss << "]";
    if (mat.rows() > max_rows || mat.cols() > max_cols) {
        oss << " ... (" << mat.rows() << "x" << mat.cols() << ")";
    }
    return oss.str();
}

// ===================================================================
// 断言宏（调试用）
// ===================================================================

#ifdef DEBUG
#define MATH_SAFETY_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "[MathSafety] Assertion failed: " << message \
                      << " at " << __FILE__ << ":" << __LINE__ \
                      << std::endl; \
            std::abort(); \
        } \
    } while(0)
#else
#define MATH_SAFETY_ASSERT(condition, message) ((void)0)
#endif

#define MATH_SAFETY_ASSERT_FINITE(x) \
    MATH_SAFETY_ASSERT(isFinite(x), "Value is not finite: " + toString(x))

#define MATH_SAFETY_ASSERT_POSITIVE(x) \
    MATH_SAFETY_ASSERT(isPositive(x), "Value is not positive: " + toString(x))

#define MATH_SAFETY_ASSERT_VALID_TIMESTAMP(x) \
    MATH_SAFETY_ASSERT(isValidTimestamp(x), "Invalid timestamp: " + toString(x))

// ===================================================================
// 性能优化辅助
// ===================================================================

/**
 * @brief 快速向量和点积（避免临时对象）
 */
inline double fastDot(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/**
 * @brief 快速向量范数平方
 */
inline double fastNormSq(const Eigen::Vector3d& v) {
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}

} // namespace MathSafety
} // namespace ns_unicalib
