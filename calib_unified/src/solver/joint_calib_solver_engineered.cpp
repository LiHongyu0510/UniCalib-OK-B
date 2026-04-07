/**
 * UniCalib — 联合标定求解器工程化版本（示例实现）
 * 集成错误码、日志增强、性能计时、健康监控、诊断工具
 * 
 * 这是一个示例实现，展示如何在现有代码基础上集成新的工程化基础设施
 */

#include "unicalib/solver/joint_calib_solver.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/timing.h"
#include "unicalib/common/health_monitor.h"
#include "unicalib/common/diagnostics.h"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <thread>
#include <fmt/core.h>
#include <fmt/format.h>

namespace fs = std::filesystem;
namespace ns_unicalib {

// ===================================================================
// 主标定函数 - 工程化版本
// ===================================================================
Status JointCalibSolver::calibrate(const CalibDataBundle& data, 
                                  CalibSummary& summary) {
    
    // 初始化日志（如果尚未初始化）
    if (!Logger::isInitialized()) {
        LoggerConfig log_config;
        log_config.logger_name = "UniCalib";
        log_config.console_level = spdlog::level::info;
        log_config.file_level = spdlog::level::trace;
        log_config.log_file = cfg_.output_dir + "/calibration.log";
        log_config.use_rotating_file = true;
        log_config.max_file_size_mb = 10;
        log_config.max_files = 3;
        Logger::init(log_config);
        
        UNICALIB_INFO("=== UniCalib Calibration Engine (v2.0) ===");
        UNICALIB_INFO("Logging initialized: {}", log_config.log_file);
    }
    
    // 注册健康监控模块
    HealthMonitor& health = HealthMonitor::instance();
    health.registerModule("joint_calib_solver");
    health.registerModule("data_pipeline");
    health.registerModule("imu_intrinsic");
    health.registerModule("camera_intrinsic");
    health.registerModule("extrinsic_init");
    health.registerModule("joint_optimization");
    
    // 设置诊断级别
    auto& diag = DiagnosticsEngine::instance();
    diag.setLevel(DiagnosticsLevel::STANDARD);
    
    // 初始化摘要
    summary = CalibSummary{};
    summary.params = params_;
    summary.success = false;
    summary.error_code = ErrorCode::SUCCESS;
    
    // 整体性能计时
    UNICALIB_SCOPED_TIMER("JointCalibration");
    
    // 阶段计时
    auto t_start = Clock::now();
    
    UNICALIB_INFO("\n========== 开始联合标定 ==========");
    UNICALIB_INFO("配置: IMU内参={}, 相机内参={}, IMU-LiDAR={}, LiDAR-Camera={}, Camera-Camera={}, B样条精化={}",
                 cfg_.do_imu_intrinsic, cfg_.do_camera_intrinsic,
                 cfg_.do_imu_lidar_extrinsic, cfg_.do_lidar_camera_extrinsic,
                 cfg_.do_cam_cam_extrinsic, cfg_.do_joint_bspline_refine);
    UNICALIB_INFO("输出目录: {}", cfg_.output_dir);
    
    // 数据验证
    UNICALIB_SCOPED_TIMER("DataValidation");
    if (data.empty()) {
        UNICALIB_ERROR("输入数据为空");
        summary.error_code = ErrorCode::EMPTY_DATA;
        summary.error_message = "Input data bundle is empty";
        health.heartbeat("joint_calib_solver", HealthStatus::FAILED, 
                        "Empty input data");
        return Status::Error(ErrorCode::EMPTY_DATA, "Input data bundle is empty");
    }
    
    // 检查必需的传感器数据
    if (cfg_.do_imu_intrinsic && data.imu_data.empty()) {
        UNICALIB_ERROR("启用IMU内参标定但无IMU数据");
        summary.error_code = ErrorCode::INSUFFICIENT_DATA;
        summary.error_message = "IMU intrinsic calibration enabled but no IMU data";
        return Status::Error(ErrorCode::INSUFFICIENT_DATA, 
                            "IMU intrinsic calibration enabled but no IMU data");
    }
    
    if (cfg_.do_camera_intrinsic && data.camera_image_paths.empty()) {
        UNICALIB_ERROR("启用相机内参标定但无相机图像");
        summary.error_code = ErrorCode::INSUFFICIENT_DATA;
        summary.error_message = "Camera intrinsic calibration enabled but no images";
        return Status::Error(ErrorCode::INSUFFICIENT_DATA,
                            "Camera intrinsic calibration enabled but no images");
    }
    
    health.heartbeat("joint_calib_solver", HealthStatus::HEALTHY);
    health.heartbeat("data_pipeline", HealthStatus::HEALTHY);
    
    // 收集系统诊断信息
    diag.collectSystemInfo();
    
    try {
        // ===================================================================
        // Phase 1: 内参标定
        // ===================================================================
        if (cfg_.do_imu_intrinsic || cfg_.do_camera_intrinsic) {
            UNICALIB_INFO("\n--- Phase 1: 内参标定 ---");
            auto phase1_start = Clock::now();
            
            phase1_intrinsics(data, summary);
            
            auto phase1_duration = TimingUtils::toMilliseconds(
                Clock::now() - phase1_start);
            summary.phase_durations_ms["Phase1-Intrinsics"] = phase1_duration;
            UNICALIB_INFO("Phase 1 完成: {:.3f} ms", phase1_duration);
        }
        
        // 检查 Phase 1 质量
        auto phase1_status = health.getStatus("imu_intrinsic");
        if (phase1_status && *phase1_status != HealthStatus::HEALTHY) {
            UNICALIB_WARN("IMU内参标定质量: {}", 
                         healthStatusName(*phase1_status));
        }
        
        // ===================================================================
        // Phase 2: 粗外参初始化
        // ===================================================================
        if (cfg_.do_imu_lidar_extrinsic || cfg_.do_lidar_camera_extrinsic || 
            cfg_.do_cam_cam_extrinsic) {
            UNICALIB_INFO("\n--- Phase 2: 粗外参初始化 ---");
            auto phase2_start = Clock::now();
            
            phase2_coarse_extrinsic(data, summary);
            
            auto phase2_duration = TimingUtils::toMilliseconds(
                Clock::now() - phase2_start);
            summary.phase_durations_ms["Phase2-CoarseExtrinsic"] = phase2_duration;
            UNICALIB_INFO("Phase 2 完成: {:.3f} ms", phase2_duration);
        }
        
        // ===================================================================
        // Phase 3: 联合B样条优化
        // ===================================================================
        if (cfg_.do_joint_bspline_refine) {
            UNICALIB_INFO("\n--- Phase 3: 联合B样条精优化 ---");
            auto phase3_start = Clock::now();
            
            // 检查数据量是否足够
            size_t total_frames = 0;
            for (const auto& [id, scans] : data.lidar_scans) {
                total_frames += scans.size();
            }
            if (total_frames < 100) {
                UNICALIB_WARN("LiDAR帧数不足 ({}), 跳过B样条优化", total_frames);
                health.heartbeat("joint_optimization", HealthStatus::WARNING,
                              "Insufficient LiDAR frames for B-spline refinement");
            } else {
                phase3_joint_refine(data, summary);
                health.heartbeat("joint_optimization", HealthStatus::HEALTHY);
            }
            
            auto phase3_duration = TimingUtils::toMilliseconds(
                Clock::now() - phase3_start);
            summary.phase_durations_ms["Phase3-JointRefine"] = phase3_duration;
            UNICALIB_INFO("Phase 3 完成: {:.3f} ms", phase3_duration);
        }
        
        // ===================================================================
        // Phase 4: 验证与报告生成
        // ===================================================================
        UNICALIB_INFO("\n--- Phase 4: 验证与报告 ---");
        auto phase4_start = Clock::now();
        
        phase4_validation(data, summary);
        
        auto phase4_duration = TimingUtils::toMilliseconds(
            Clock::now() - phase4_start);
        summary.phase_durations_ms["Phase4-Validation"] = phase4_duration;
        UNICALIB_INFO("Phase 4 完成: {:.3f} ms", phase4_duration);
        
        // 整体状态
        summary.success = true;
        summary.error_code = ErrorCode::SUCCESS;
        
        health.heartbeat("joint_calib_solver", HealthStatus::HEALTHY,
                       "Calibration completed successfully");
        
    } catch (const ConfigException& e) {
        // 配置错误
        summary.success = false;
        summary.error_code = e.code();
        summary.error_message = e.what();
        UNICALIB_ERROR("配置错误: {}", e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::CRITICAL, e.what());
        
        return Status::Error(e.code(), e.what());
        
    } catch (const DataException& e) {
        // 数据错误
        summary.success = false;
        summary.error_code = e.code();
        summary.error_message = e.what();
        UNICALIB_ERROR("数据错误: {}", e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::CRITICAL, e.what());
        
        return Status::Error(e.code(), e.what());
        
    } catch (const CalibException& e) {
        // 标定算法错误
        summary.success = false;
        summary.error_code = e.code();
        summary.error_message = e.what();
        UNICALIB_ERROR("标定错误 [{}]: {}", e.codeValue(), e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::CRITICAL, e.what());
        
        // 降级：保留当前 params_（可能含 Phase2 粗外参），调用方可通过 summary.params 或 get_params() 获取
        summary.params = params_;
        UNICALIB_WARN("标定异常，返回已得到的参数（若有 Phase2 粗外参可继续使用）");
        return Status::Error(e.code(), e.what());
        
    } catch (const NumericalException& e) {
        // 数值计算错误
        summary.success = false;
        summary.error_code = e.code();
        summary.error_message = e.what();
        UNICALIB_ERROR("数值错误: {}", e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::CRITICAL, e.what());
        
        return Status::Error(e.code(), e.what());
        
    } catch (const SystemException& e) {
        // 系统错误
        summary.success = false;
        summary.error_code = e.code();
        summary.error_message = e.what();
        UNICALIB_CRITICAL("系统错误: {}", e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::FAILED, e.what());
        
        return Status::Error(e.code(), e.what());
        
    } catch (const UniCalibException& e) {
        // 通用异常
        summary.success = false;
        summary.error_code = e.code();
        summary.error_message = e.what();
        UNICALIB_ERROR("UniCalib异常 [{}]: {}", e.codeValue(), e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::CRITICAL, e.what());
        
        return Status::Error(e.code(), e.what());
        
    } catch (const std::exception& e) {
        // 其他异常
        summary.success = false;
        summary.error_code = ErrorCode::INTERNAL_ERROR;
        summary.error_message = e.what();
        UNICALIB_CRITICAL("未处理的异常: {}", e.what());
        health.heartbeat("joint_calib_solver", HealthStatus::FAILED, e.what());
        
        return Status::Error(ErrorCode::INTERNAL_ERROR, 
                            fmt::format("Unhandled exception: {}", e.what()));
        
    } catch (...) {
        // 未知异常
        summary.success = false;
        summary.error_code = ErrorCode::INTERNAL_ERROR;
        summary.error_message = "Unknown exception";
        UNICALIB_CRITICAL("未知异常");
        health.heartbeat("joint_calib_solver", HealthStatus::FAILED, "Unknown exception");
        
        return Status::Error(ErrorCode::INTERNAL_ERROR, "Unknown exception");
    }
    
    // 保存结果
    UNICALIB_SCOPED_TIMER("SaveResults");
    save_results(summary);
    
    // 计算总耗时
    auto t_end = Clock::now();
    summary.total_duration_ms = TimingUtils::toMilliseconds(t_end - t_start);
    
    UNICALIB_INFO("\n========== 联合标定完成 ==========");
    UNICALIB_INFO("总耗时: {:.3f} s ({:.3f} ms)",
                 summary.total_duration_ms / 1000.0,
                 summary.total_duration_ms);
    UNICALIB_INFO("成功: {}", summary.success ? "是" : "否");
    if (!summary.success) {
        UNICALIB_ERROR("错误码: {} ({})",
                      summary.error_code,
                      errorCodeName(summary.error_code));
        UNICALIB_ERROR("错误信息: {}", summary.error_message);
    }
    
    // 打印各阶段耗时
    UNICALIB_INFO("阶段耗时:");
    for (const auto& [phase, duration_ms] : summary.phase_durations_ms) {
        UNICALIB_INFO("  {}: {:.3f} ms", phase, duration_ms);
    }
    
    // 生成诊断报告
    if (cfg_.generate_html_report) {
        diag.checkPerformance();
        std::string diagnostics_path = cfg_.output_dir + "/diagnostics_report.txt";
        diag.saveReport(diagnostics_path);
        summary.diagnostics_path = diagnostics_path;
        UNICALIB_INFO("诊断报告: {}", diagnostics_path);
    }
    
    // 生成健康报告
    UNICALIB_INFO("\n{}", health.generateReport());
    
    // 性能分析报告
    UNICALIB_INFO("\n{}", PerformanceProfiler::instance().getStats("JointCalibration").summary());
    
    params_->print_summary();
    return Status::OK();
}

// ===================================================================
// 辅助函数：报告进度（带健康监控）
// ===================================================================
void JointCalibSolver::report_progress(const std::string& stage, 
                                     double progress,
                                     const std::string& msg) {
    
    std::string progress_msg = fmt::format("[{}] {:.1f}% - {}", stage, progress * 100, msg);
    
    UNICALIB_LOG_STEP(stage, "{:.1f}% - {}", progress * 100, msg);
    
    // 更新健康监控
    HealthMonitor::instance().heartbeat("joint_calib_solver", 
                                      HealthStatus::HEALTHY, progress_msg);
    
    // 调用用户回调
    if (progress_cb_) {
        progress_cb_(stage, progress, msg);
    }
}

// ===================================================================
// 阶段增强包装器示例
// ===================================================================
void phase1_intrinsics_enhanced(
    const CalibDataBundle& data, 
    CalibSummary& summary) {
    
    UNICALIB_SCOPED_TIMER("Phase1-Intrinsics");
    UNICALIB_SCOPED_PROFILE("imu_intrinsic");
    
    // 检查数据质量
    std::vector<double> imu_timestamps;
    std::vector<Eigen::Vector3d> imu_data;
    
    for (const auto& [sensor_id, frames] : data.imu_data) {
        for (const auto& frame : frames) {
            imu_timestamps.push_back(frame.timestamp);
            imu_data.push_back(frame.gyro);
            imu_data.push_back(frame.accel);
        }
    }
    
    // 数据质量诊断
    auto quality = DiagnosticsEngine::instance().checkDataQuality(
        imu_timestamps, imu_data);
    
    UNICALIB_INFO("数据质量评分: {:.1f}/100", quality.quality_score);
    
    if (quality.quality_score < 50.0) {
        UNICALIB_WARN("数据质量较低，标定结果可能不理想");
        HealthMonitor::instance().heartbeat("imu_intrinsic", 
                                          HealthStatus::WARNING,
                                          "Low data quality");
    } else {
        HealthMonitor::instance().heartbeat("imu_intrinsic", 
                                          HealthStatus::HEALTHY);
    }
    
    // 执行IMU内参标定...
    // (实际标定代码在此)
}

}  // namespace ns_unicalib
