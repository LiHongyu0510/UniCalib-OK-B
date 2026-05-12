/**
 * UniCalib Unified — 统一错误码定义
 * 
 * 目的：
 *   - 为所有模块提供统一的错误码枚举
 *   - 支持错误分类和精确定位
 *   - 便于上层调用者精确判断失败原因
 * 
 * 分类：
 *   - 1xxx: 配置/参数错误
 *   - 2xxx: 数据加载/IO错误
 *   - 3xxx: 标定算法错误
 *   - 4xxx: 数值计算错误
 *   - 5xxx: 系统/资源错误
 *   - 6xxx: 可视化/输出错误
 */

#pragma once

#include <string>
#include <magic_enum/magic_enum.hpp>

namespace ns_unicalib {

// ===================================================================
// 错误码枚举定义
// ===================================================================
enum class ErrorCode : int {
    // ─────────────────────────────────────────────────────────────
    // 成功
    // ─────────────────────────────────────────────────────────────
    SUCCESS                     = 0,
    
    // ─────────────────────────────────────────────────────────────
    // 1xxx: 配置/参数错误
    // ─────────────────────────────────────────────────────────────
    INVALID_CONFIG              = 1001,  // 配置文件格式错误
    MISSING_REQUIRED_PARAM      = 1002,  // 缺少必要参数
    INVALID_PARAM_VALUE         = 1003,  // 参数值无效
    PARAM_OUT_OF_RANGE          = 1004,  // 参数超出范围
    INCOMPATIBLE_CONFIG         = 1005,  // 配置不兼容
    UNKNOWN_CALIB_TASK          = 1006,  // 未知的标定任务类型
    INVALID_SENSOR_ID           = 1007,  // 无效的传感器ID
    
    // ─────────────────────────────────────────────────────────────
    // 2xxx: 数据加载/IO错误
    // ─────────────────────────────────────────────────────────────
    FILE_NOT_FOUND              = 2001,  // 文件不存在
    FILE_READ_ERROR             = 2002,  // 文件读取失败
    FILE_WRITE_ERROR            = 2003,  // 文件写入失败
    DATA_LOAD_FAILED            = 2004,  // 数据加载失败
    DATA_PARSE_ERROR            = 2005,  // 数据解析错误
    UNSUPPORTED_FORMAT          = 2006,  // 不支持的格式
    ROSBAG_READ_ERROR           = 2007,  // ROSBag读取错误
    TIMESTAMP_ORDER_ERROR       = 2008,  // 时间戳顺序错误
    INSUFFICIENT_DATA           = 2009,  // 数据量不足
    EMPTY_DATA                  = 2010,  // 空数据
    DATA_CORRUPTION             = 2011,  // 数据损坏
    NO_VALID_FRAMES             = 2012,  // 无有效帧
    
    // ─────────────────────────────────────────────────────────────
    // 3xxx: 标定算法错误
    // ─────────────────────────────────────────────────────────────
    CALIBRATION_FAILED          = 3001,  // 标定失败
    INITIALIZATION_FAILED       = 3002,  // 初始化失败
    FEATURE_DETECTION_FAILED    = 3003,  // 特征检测失败
    FEATURE_MATCHING_FAILED     = 3004,  // 特征匹配失败
    POSE_ESTIMATION_FAILED      = 3005,  // 位姿估计失败
    OPTIMIZATION_DIVERGED       = 3006,  // 优化发散
    OPTIMIZATION_TIMEOUT        = 3007,  // 优化超时
    CONVERGENCE_FAILED          = 3008,  // 收敛失败
    SPLINE_INIT_FAILED          = 3009,  // 样条初始化失败
    ALIGNMENT_FAILED            = 3010,  // 对齐失败
    EXTRINSIC_INIT_FAILED       = 3011,  // 外参初始化失败
    INTRINSIC_INIT_FAILED       = 3012,  // 内参初始化失败
    INSUFFICIENT_FEATURES       = 3013,  // 特征点不足
    DEGENERATE_CONFIGURATION    = 3014,  // 退化配置
    POOR_QUALITY_RESULT         = 3015,  // 结果质量差
    
    // ─────────────────────────────────────────────────────────────
    // 4xxx: 数值计算错误
    // ─────────────────────────────────────────────────────────────
    NUMERICAL_ERROR             = 4001,  // 数值计算错误
    DIVISION_BY_ZERO            = 4002,  // 除零错误
    SINGULAR_MATRIX             = 4003,  // 奇异矩阵
    NAN_DETECTED                = 4004,  // 检测到NaN
    INF_DETECTED                = 4005,  // 检测到Inf
    CERES_SOLVER_ERROR          = 4006,  // Ceres求解器错误
    CHOLESKY_FAILED             = 4007,  // Cholesky分解失败
    QR_FAILED                   = 4008,  // QR分解失败
    SVD_FAILED                  = 4009,  // SVD分解失败
    QUATERNION_NORMALIZE_FAILED = 4010,  // 四元数归一化失败
    
    // ─────────────────────────────────────────────────────────────
    // 5xxx: 系统/资源错误
    // ─────────────────────────────────────────────────────────────
    OUT_OF_MEMORY               = 5001,  // 内存不足
    THREAD_ERROR                = 5002,  // 线程错误
    RESOURCE_UNAVAILABLE        = 5003,  // 资源不可用
    PYTHON_BRIDGE_ERROR         = 5004,  // Python桥接错误
    GPU_ERROR                   = 5005,  // GPU错误
    OPENGL_ERROR                = 5006,  // OpenGL错误
    PCL_ERROR                   = 5007,  // PCL错误
    OPENCV_ERROR                = 5008,  // OpenCV错误
    INTERNAL_ERROR              = 5009,  // 内部错误
    UNIMPLEMENTED               = 5010,  // 未实现
    
    // ─────────────────────────────────────────────────────────────
    // 6xxx: 可视化/输出错误
    // ─────────────────────────────────────────────────────────────
    VISUALIZATION_ERROR         = 6001,  // 可视化错误
    REPORT_GENERATION_FAILED    = 6002,  // 报告生成失败
    EXPORT_FAILED               = 6003,  // 导出失败
    DISPLAY_INIT_FAILED         = 6004,  // 显示初始化失败
};

// ===================================================================
// 错误码工具函数
// ===================================================================

/**
 * @brief 获取错误码的字符串名称
 */
inline std::string errorCodeName(ErrorCode code) {
    return std::string(magic_enum::enum_name(code));
}

/**
 * @brief 获取错误码的数值
 */
inline int errorCodeValue(ErrorCode code) {
    return static_cast<int>(code);
}

/**
 * @brief 获取错误码所属的分类
 * @return 分类名称
 */
inline std::string errorCodeCategory(ErrorCode code) {
    int val = static_cast<int>(code);
    if (val == 0) return "SUCCESS";
    if (val >= 1001 && val < 2000) return "CONFIG";
    if (val >= 2001 && val < 3000) return "IO";
    if (val >= 3001 && val < 4000) return "CALIBRATION";
    if (val >= 4001 && val < 5000) return "NUMERICAL";
    if (val >= 5001 && val < 6000) return "SYSTEM";
    if (val >= 6001 && val < 7000) return "VISUALIZATION";
    return "UNKNOWN";
}

/**
 * @brief 获取错误码的可读描述
 */
inline std::string errorCodeDescription(ErrorCode code) {
    switch (code) {
        // 成功
        case ErrorCode::SUCCESS:
            return "操作成功";
            
        // 配置/参数错误
        case ErrorCode::INVALID_CONFIG:
            return "配置文件格式错误或配置无效";
        case ErrorCode::MISSING_REQUIRED_PARAM:
            return "缺少必要的参数";
        case ErrorCode::INVALID_PARAM_VALUE:
            return "参数值无效";
        case ErrorCode::PARAM_OUT_OF_RANGE:
            return "参数超出允许范围";
        case ErrorCode::INCOMPATIBLE_CONFIG:
            return "配置项之间存在冲突或不兼容";
        case ErrorCode::UNKNOWN_CALIB_TASK:
            return "未知的标定任务类型";
        case ErrorCode::INVALID_SENSOR_ID:
            return "无效的传感器标识符";
            
        // 数据加载/IO错误
        case ErrorCode::FILE_NOT_FOUND:
            return "指定的文件不存在";
        case ErrorCode::FILE_READ_ERROR:
            return "文件读取失败";
        case ErrorCode::FILE_WRITE_ERROR:
            return "文件写入失败";
        case ErrorCode::DATA_LOAD_FAILED:
            return "数据加载失败";
        case ErrorCode::DATA_PARSE_ERROR:
            return "数据解析错误，格式可能不正确";
        case ErrorCode::UNSUPPORTED_FORMAT:
            return "不支持的数据格式";
        case ErrorCode::ROSBAG_READ_ERROR:
            return "ROS bag文件读取错误";
        case ErrorCode::TIMESTAMP_ORDER_ERROR:
            return "时间戳未按顺序排列或存在重复";
        case ErrorCode::INSUFFICIENT_DATA:
            return "数据量不足以完成标定";
        case ErrorCode::EMPTY_DATA:
            return "数据为空";
        case ErrorCode::DATA_CORRUPTION:
            return "数据损坏或格式异常";
        case ErrorCode::NO_VALID_FRAMES:
            return "没有有效的数据帧";
            
        // 标定算法错误
        case ErrorCode::CALIBRATION_FAILED:
            return "标定过程失败";
        case ErrorCode::INITIALIZATION_FAILED:
            return "算法初始化失败";
        case ErrorCode::FEATURE_DETECTION_FAILED:
            return "特征点检测失败";
        case ErrorCode::FEATURE_MATCHING_FAILED:
            return "特征匹配失败";
        case ErrorCode::POSE_ESTIMATION_FAILED:
            return "位姿估计失败";
        case ErrorCode::OPTIMIZATION_DIVERGED:
            return "优化过程发散";
        case ErrorCode::OPTIMIZATION_TIMEOUT:
            return "优化过程超时";
        case ErrorCode::CONVERGENCE_FAILED:
            return "优化未能收敛";
        case ErrorCode::SPLINE_INIT_FAILED:
            return "B样条轨迹初始化失败";
        case ErrorCode::ALIGNMENT_FAILED:
            return "传感器对齐失败";
        case ErrorCode::EXTRINSIC_INIT_FAILED:
            return "外参初始化失败";
        case ErrorCode::INTRINSIC_INIT_FAILED:
            return "内参初始化失败";
        case ErrorCode::INSUFFICIENT_FEATURES:
            return "检测到的特征点数量不足";
        case ErrorCode::DEGENERATE_CONFIGURATION:
            return "传感器配置退化（如纯旋转运动）";
        case ErrorCode::POOR_QUALITY_RESULT:
            return "标定结果质量不佳";
            
        // 数值计算错误
        case ErrorCode::NUMERICAL_ERROR:
            return "数值计算错误";
        case ErrorCode::DIVISION_BY_ZERO:
            return "除零错误";
        case ErrorCode::SINGULAR_MATRIX:
            return "矩阵奇异或接近奇异";
        case ErrorCode::NAN_DETECTED:
            return "计算过程中检测到NaN值";
        case ErrorCode::INF_DETECTED:
            return "计算过程中检测到Inf值";
        case ErrorCode::CERES_SOLVER_ERROR:
            return "Ceres求解器内部错误";
        case ErrorCode::CHOLESKY_FAILED:
            return "Cholesky分解失败";
        case ErrorCode::QR_FAILED:
            return "QR分解失败";
        case ErrorCode::SVD_FAILED:
            return "SVD分解失败";
        case ErrorCode::QUATERNION_NORMALIZE_FAILED:
            return "四元数归一化失败（可能为零四元数）";
            
        // 系统/资源错误
        case ErrorCode::OUT_OF_MEMORY:
            return "内存不足";
        case ErrorCode::THREAD_ERROR:
            return "线程创建或同步错误";
        case ErrorCode::RESOURCE_UNAVAILABLE:
            return "请求的资源不可用";
        case ErrorCode::PYTHON_BRIDGE_ERROR:
            return "Python子进程桥接错误";
        case ErrorCode::GPU_ERROR:
            return "GPU相关错误";
        case ErrorCode::OPENGL_ERROR:
            return "OpenGL渲染错误";
        case ErrorCode::PCL_ERROR:
            return "PCL点云库错误";
        case ErrorCode::OPENCV_ERROR:
            return "OpenCV图像处理错误";
        case ErrorCode::INTERNAL_ERROR:
            return "内部逻辑错误";
        case ErrorCode::UNIMPLEMENTED:
            return "该功能尚未实现";
            
        // 可视化/输出错误
        case ErrorCode::VISUALIZATION_ERROR:
            return "可视化渲染错误";
        case ErrorCode::REPORT_GENERATION_FAILED:
            return "标定报告生成失败";
        case ErrorCode::EXPORT_FAILED:
            return "结果导出失败";
        case ErrorCode::DISPLAY_INIT_FAILED:
            return "显示窗口初始化失败";
            
        default:
            return "未知错误";
    }
}

/**
 * @brief 判断错误码是否为成功
 */
inline bool isSuccess(ErrorCode code) {
    return code == ErrorCode::SUCCESS;
}

/**
 * @brief 判断错误码是否为可恢复错误
 */
inline bool isRecoverable(ErrorCode code) {
    int val = static_cast<int>(code);
    // 配置错误、数据不足等可以尝试恢复
    return (val >= 1001 && val < 3000) ||
           code == ErrorCode::INSUFFICIENT_FEATURES ||
           code == ErrorCode::POOR_QUALITY_RESULT;
}

/**
 * @brief 判断错误码是否为致命错误
 */
inline bool isFatal(ErrorCode code) {
    return code == ErrorCode::OUT_OF_MEMORY ||
           code == ErrorCode::INTERNAL_ERROR ||
           code == ErrorCode::RESOURCE_UNAVAILABLE;
}

}  // namespace ns_unicalib
