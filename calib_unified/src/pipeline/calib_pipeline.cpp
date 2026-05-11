/**
 * UniCalib Unified — CalibPipeline 实现
 *
 * 核心流程:
 *   1. 为每个 stage 创建独立日志文件 (output_dir/logs/<stage>_<task>_<ts>.log)
 *   2. 记录阶段开始/结束时间、残差、收敛状态
 *   3. 汇总报告写入 output_dir/pipeline_report_<ts>.yaml
 */

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <optional>
#include <chrono>
#include <thread>

// ROS2 头必须在 namespace 外包含，避免 sensor_msgs 等被注入 ns_unicalib 导致编译错误
#include "unicalib/io/ros2_data_source.h"
#include "unicalib/io/new_format_path.h"

#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/viz/calib_visualizer.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/accuracy_logger.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/io/yaml_io.h"
#include <Eigen/SVD>
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ---------------------------------------------------------------------------
// 工具: 将 3x3 矩阵投影到最近的正交旋转 (SO3)，避免非正交初值导致 Sophus 断言/崩溃
// ---------------------------------------------------------------------------
static Eigen::Matrix3d project_to_rotation(const Eigen::Matrix3d& M) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) R.col(2) *= -1;
    return R;
}

// ---------------------------------------------------------------------------
// 工具: 从 YAML 文件加载 T_Imu_Lidar / T_Body_Lidar 4x4 得到 SE3（用于 IMU-LiDAR 初值）
// ---------------------------------------------------------------------------
static std::optional<Sophus::SE3d> load_se3_from_imu_lidar_yaml(const std::string& path) {
    try {
        YAML::Node n = YAML::LoadFile(path);
        YAML::Node t_node;
        if (n["T_Imu_Lidar"] && n["T_Imu_Lidar"]["data"]) t_node = n["T_Imu_Lidar"];
        else if (n["T_Body_Lidar"] && n["T_Body_Lidar"]["data"]) t_node = n["T_Body_Lidar"];
        if (!t_node || !t_node["data"].IsSequence() || t_node["data"].size() < 16) return std::nullopt;
        const auto& data = t_node["data"];
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                T(i, j) = data[i * 4 + j].as<double>();
        Eigen::Matrix3d R = project_to_rotation(T.block<3,3>(0,0));
        return Sophus::SE3d(R, T.block<3,1>(0,3));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

// ---------------------------------------------------------------------------
// 工具: 获取当前时间字符串
// ---------------------------------------------------------------------------
static std::string now_str() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ---------------------------------------------------------------------------
// 工具: CalibTaskType → 字符串
// ---------------------------------------------------------------------------
static std::string task_str(CalibTaskType t) {
    switch (t) {
        case CalibTaskType::IMU_INTRINSIC:     return "imu_intrinsic";
        case CalibTaskType::CAM_INTRINSIC:     return "cam_intrinsic";
        case CalibTaskType::IMU_LIDAR_EXTRIN:  return "imu_lidar_extrin";
        case CalibTaskType::LIDAR_LIDAR_EXTRIN: return "lidar_lidar_extrin";
        case CalibTaskType::LIDAR_CAM_EXTRIN: return "lidar_cam_extrin";
        case CalibTaskType::CAM_CAM_EXTRIN:   return "cam_cam_extrin";
        case CalibTaskType::ALL:              return "all";
        default:                              return "none";
    }
}

// ===========================================================================
// PipelineReport
// ===========================================================================

void PipelineReport::print_summary() const {
    UNICALIB_INFO("====== Pipeline Report: {} ======", pipeline_id);
    UNICALIB_INFO("  Total time: {:.1f} ms", total_elapsed_ms());
    UNICALIB_INFO("  All stages succeeded: {}", all_converged() ? "YES" : "NO");
    int need_manual_count = 0;
    for (const auto& r : stage_results)
        if (r.needs_manual_refine()) need_manual_count++;
    if (need_manual_count > 0)
        UNICALIB_WARN("  标定质量: {} 项 RMS 超过阈值，建议手动校准或检查数据 (见下方 ↳)",
                      need_manual_count);
    else if (!stage_results.empty())
        UNICALIB_INFO("  标定质量: 各阶段 RMS 均在阈值内");
    for (const auto& r : stage_results) {
        UNICALIB_INFO("  [{}-{}] success={} rms={:.4f} time={:.1f}ms: {}",
                      stage_name(r.stage), task_str(r.task),
                      r.success, r.residual_rms, r.elapsed_ms, r.message);
        if (r.needs_manual_refine()) {
            UNICALIB_WARN("    ↳ RMS {:.4f} > threshold {:.4f} — 建议手动校准",
                          r.residual_rms, r.quality_threshold);
        }
    }
    if (final_params) {
        final_params->print_summary();
    }
    UNICALIB_INFO("================================");
}

void PipelineReport::save_report(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_WARN("Cannot write pipeline report to: {}", path);
        return;
    }
    f << "pipeline_id: " << pipeline_id << "\n";
    f << "total_elapsed_ms: " << total_elapsed_ms() << "\n";
    f << "all_converged: " << all_converged() << "\n";
    f << "stages:\n";
    for (const auto& r : stage_results) {
        f << "  - stage: " << stage_name(r.stage) << "\n";
        f << "    task: " << task_str(r.task) << "\n";
        f << "    success: " << r.success << "\n";
        f << "    residual_rms: " << r.residual_rms << "\n";
        f << "    elapsed_ms: " << r.elapsed_ms << "\n";
        f << "    message: \"" << r.message << "\"\n";
        if (!r.log_file.empty())
            f << "    log_file: " << r.log_file << "\n";
        if (r.needs_manual_refine())
            f << "    needs_manual_refine: true\n";
    }
    UNICALIB_INFO("Pipeline report saved: {}", path);
}

// ===========================================================================
// CalibPipeline
// ===========================================================================

CalibPipeline::CalibPipeline()
    : CalibPipeline(PipelineConfig{}) {}

CalibPipeline::CalibPipeline(const PipelineConfig& cfg)
    : cfg_(cfg)
    , params_(CalibParamManager::Create())
{
    // 设置全局日志级别
    spdlog::level::level_enum lv = spdlog::level::info;
    if      (cfg_.log_level == "trace")    lv = spdlog::level::trace;
    else if (cfg_.log_level == "debug")    lv = spdlog::level::debug;
    else if (cfg_.log_level == "warn")     lv = spdlog::level::warn;
    else if (cfg_.log_level == "error")    lv = spdlog::level::err;

    spdlog::set_level(lv);

    // 确保输出目录存在
    fs::create_directories(cfg_.output_dir);
    fs::create_directories(cfg_.output_dir + "/logs");

    // 生成 pipeline_id
    report_.pipeline_id = "pipeline_" + now_str();
    UNICALIB_INFO("[CalibPipeline] 初始化 ID={}", report_.pipeline_id);
    UNICALIB_INFO("[CalibPipeline] 输出目录: {}", cfg_.output_dir);
    UNICALIB_INFO("[CalibPipeline] 无目标优先: {}", cfg_.prefer_targetfree);
    UNICALIB_INFO("[CalibPipeline] 任务掩码: 0x{:02X}",
                  static_cast<uint32_t>(cfg_.tasks));
}

// ---------------------------------------------------------------------------
// 阶段日志
// ---------------------------------------------------------------------------

std::string CalibPipeline::make_stage_log_path(CalibStage stage,
                                                CalibTaskType task) const {
    std::string logs_dir = resolve_logs_dir(cfg_.output_dir);
    return logs_dir + "/" + std::string(stage_name(stage)) + "_" +
           task_str(task) + "_" + now_str() + ".log";
}

void CalibPipeline::setup_stage_logger(const std::string& log_path) {
    try {
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            log_path, true);
        file_sink->set_level(spdlog::level::trace);
        // 与统一日志一致：每条记录带完整日期时间
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v");

        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::get_level());
        console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

        auto logger = std::make_shared<spdlog::logger>(
            "stage", spdlog::sinks_init_list{console_sink, file_sink});
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
    } catch (const std::exception& e) {
        // 日志创建失败不应中断流程，使用 WARN 级别
        UNICALIB_WARN("创建阶段日志失败: {}", e.what());
        // 继续执行，将使用默认日志
    }
}

void CalibPipeline::log_stage_begin(CalibStage stage, CalibTaskType task,
                                     const std::string& detail) {
    stage_start_ = Clock::now();
    UNICALIB_INFO("");
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("▶ 开始阶段: [{}] 任务: [{}]",
                  stage_name(stage), task_str(task));
    if (!detail.empty()) UNICALIB_INFO("  描述: {}", detail);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void CalibPipeline::log_stage_end(const StageResult& result) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       Clock::now() - stage_start_).count();
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("■ 结束阶段: [{}] 任务: [{}]",
                  stage_name(result.stage), task_str(result.task));
    UNICALIB_INFO("  结果: {} | 残差RMS: {:.4f} | 耗时: {} ms",
                  result.success ? "✓ 成功" : "✗ 失败",
                  result.residual_rms, elapsed);
    UNICALIB_INFO("  消息: {}", result.message);
    UNICALIB_CALC("阶段结束 stage={} task={} success={} residual_rms={:.6f} elapsed_ms={:.1f} threshold={:.4f}",
                  stage_name(result.stage), task_str(result.task), result.success,
                  result.residual_rms, result.elapsed_ms, result.quality_threshold);
    if (result.needs_manual_refine()) {
        UNICALIB_WARN("  ⚠ 自动标定精度不足 (RMS={:.4f} > 阈值={:.4f})",
                      result.residual_rms, result.quality_threshold);
        UNICALIB_WARN("  建议执行手动校准 (--manual 模式)");
    }
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void CalibPipeline::log_step(CalibStage stage, const std::string& step,
                              const std::string& msg,
                              spdlog::level::level_enum lv) {
    spdlog::log(lv, "[{}-{}] [{}] {}",
                stage_name(stage), step, step, msg);
    if (progress_cb_) {
        progress_cb_(stage, step, -1.0);
    }
}

void CalibPipeline::log_metric(const std::string& name, double value,
                                const std::string& unit) {
    UNICALIB_INFO("  📊 {}: {:.6f} {}", name, value, unit);
}

void CalibPipeline::log_param_change(const std::string& param_name,
                                      const std::string& before,
                                      const std::string& after) {
    UNICALIB_INFO("  🔄 参数更新 [{}]: {} → {}", param_name, before, after);
}

void CalibPipeline::set_manual_lidar_lidar_first_frame(const LiDARScan& ref_first,
                                                        const LiDARScan& target_first,
                                                        const std::string& ref_id,
                                                        const std::string& target_id) {
    manual_cache_.lidar_lidar_ref_scan = ref_first;
    manual_cache_.lidar_lidar_target_scan = target_first;
    manual_cache_.lidar_lidar_ref_id = ref_id;
    manual_cache_.lidar_lidar_target_id = target_id;
}

// ---------------------------------------------------------------------------
// 运行完整流水线
// ---------------------------------------------------------------------------

PipelineReport CalibPipeline::run() {
    UNICALIB_INFO("");
    UNICALIB_INFO("╔═══════════════════════════════════════════════╗");
    UNICALIB_INFO("║   UniCalib 两阶段标定流水线 — 开始            ║");
    UNICALIB_INFO("║   Pipeline ID: {:30s} ║", report_.pipeline_id);
    UNICALIB_INFO("╚═══════════════════════════════════════════════╝");

    const uint32_t task_mask = static_cast<uint32_t>(cfg_.tasks);
    const bool do_imu_intrin = has_task(cfg_.tasks, CalibTaskType::IMU_INTRINSIC);
    const bool do_imu_lidar  = has_task(cfg_.tasks, CalibTaskType::IMU_LIDAR_EXTRIN);
    // 未做 IMU 内参但需要 IMU-LiDAR 外参时，从 results/imu_intrinsic/ 加载 IMU 内参
    if (!do_imu_intrin && do_imu_lidar && !cfg_.imu_intrinsic_files.empty() && params_) {
        for (const auto& [imu_id, yaml_path] : cfg_.imu_intrinsic_files) {
            if (!yaml_path.empty() && fs::exists(yaml_path)) {
                try {
                    auto intrin = YamlIO::load_imu_intrinsics(yaml_path);
                    params_->imu_intrinsics[imu_id] = std::make_shared<IMUIntrinsics>(intrin);
                    UNICALIB_INFO("[Pipeline] 已从 results 加载 IMU 内参: {} <- {}", imu_id, yaml_path);
                } catch (const std::exception& e) {
                    UNICALIB_WARN("[Pipeline] 加载 IMU 内参失败 {}: {}", yaml_path, e.what());
                }
            }
        }
    }

    // 枚举所有可能的单任务
    const CalibTaskType single_tasks[] = {
        CalibTaskType::IMU_INTRINSIC,
        CalibTaskType::CAM_INTRINSIC,
        CalibTaskType::IMU_LIDAR_EXTRIN,
        CalibTaskType::LIDAR_LIDAR_EXTRIN,
        CalibTaskType::LIDAR_CAM_EXTRIN,
        CalibTaskType::CAM_CAM_EXTRIN,
    };

    for (auto task : single_tasks) {
        if (!has_task(cfg_.tasks, task)) continue;

        // ─── Stage 1: AI 粗标定 ───────────────────────────────────────────
        bool do_coarse =
            (task == CalibTaskType::CAM_INTRINSIC     && cfg_.enable_coarse_cam_intrin)  ||
            (task == CalibTaskType::IMU_INTRINSIC     && cfg_.enable_coarse_imu_intrin)  ||
            (task == CalibTaskType::IMU_LIDAR_EXTRIN && cfg_.enable_coarse_imu_lidar)   ||
            (task == CalibTaskType::LIDAR_LIDAR_EXTRIN && false)  /* LiDAR-LiDAR 无 AI 粗标定，用 NDT 粗标定 */ ||
            (task == CalibTaskType::LIDAR_CAM_EXTRIN && cfg_.enable_coarse_lidar_cam &&
             !cfg_.lidar_cam_use_config_extrinsic_only)   ||
            (task == CalibTaskType::CAM_CAM_EXTRIN   && cfg_.enable_coarse_cam_cam);

        // 增强日志：为何未执行粗标定（便于排查未传 --coarse 或配置关闭）
        if (task == CalibTaskType::LIDAR_CAM_EXTRIN) {
            UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] enable_coarse_lidar_cam={} → do_coarse={}",
                          cfg_.enable_coarse_lidar_cam, do_coarse);
            if (cfg_.lidar_cam_use_config_extrinsic_only && cfg_.enable_coarse_lidar_cam && !do_coarse)
                UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 跳过粗标定: lidar_camera.use_config_extrinsic_only=true");
            else if (!do_coarse)
                UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 跳过粗标定: 未启用 (请使用 --coarse 启用 MIAS-LCEC 粗标定)");
        }

        if (do_coarse) {
            auto log_path = make_stage_log_path(CalibStage::COARSE_AI, task);
            setup_stage_logger(log_path);
            log_stage_begin(CalibStage::COARSE_AI, task,
                            "AI模型提供初始估计值");
            StageResult r;
            try {
                r = run_coarse_stage(task);
            } catch (const UniCalibException& e) {
                r = StageResult{};
                r.stage = CalibStage::COARSE_AI;
                r.task = task;
                r.success = false;
                r.message = std::string("[") + errorCodeName(e.code()) + "] " + e.message() +
                            " (at " + e.file() + ":" + std::to_string(e.line()) + ")";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 粗标定阶段异常: {}", r.message);
            } catch (const std::exception& e) {
                r = StageResult{};
                r.stage = CalibStage::COARSE_AI;
                r.task = task;
                r.success = false;
                r.message = std::string("std::exception: ") + e.what();
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 粗标定阶段异常: {}", r.message);
            } catch (...) {
                r = StageResult{};
                r.stage = CalibStage::COARSE_AI;
                r.task = task;
                r.success = false;
                r.message = "未知异常 (非 std::exception)";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 粗标定阶段未知异常");
            }
            r.log_file = log_path;
            log_stage_end(r);
            report_.stage_results.push_back(r);
            // 错误恢复：粗标定失败不阻断流水线，精标定将使用传统初始化（手眼/无目标）
            if (!r.success && do_coarse)
                UNICALIB_INFO("[Pipeline] 粗标定未成功，精标定将使用传统方法初始化");
        }

        // ─── Stage 2: 精标定 ──────────────────────────────────────────────
        {
            auto log_path = make_stage_log_path(CalibStage::FINE_AUTO, task);
            setup_stage_logger(log_path);
            log_stage_begin(CalibStage::FINE_AUTO, task,
                            cfg_.prefer_targetfree ? "无目标优化" : "目标辅助优化");
            StageResult r;
            try {
                r = run_fine_stage(task);
            } catch (const UniCalibException& e) {
                r = StageResult{};
                r.stage = CalibStage::FINE_AUTO;
                r.task = task;
                r.success = false;
                r.message = std::string("[") + errorCodeName(e.code()) + "] " + e.message() +
                            " (at " + e.file() + ":" + std::to_string(e.line()) + ")";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 精标定阶段异常: {}", r.message);
            } catch (const std::exception& e) {
                r = StageResult{};
                r.stage = CalibStage::FINE_AUTO;
                r.task = task;
                r.success = false;
                r.message = std::string("std::exception: ") + e.what();
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 精标定阶段异常: {}", r.message);
            } catch (...) {
                r = StageResult{};
                r.stage = CalibStage::FINE_AUTO;
                r.task = task;
                r.success = false;
                r.message = "未知异常 (非 std::exception)";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 精标定阶段未知异常");
            }
            r.log_file = log_path;
            log_stage_end(r);
            report_.stage_results.push_back(r);

            // ─── Stage 3: 手动校准 (--manual 时始终进入；或 RMS 超阈值时进入) ─────
            const bool task_supports_manual = (task == CalibTaskType::LIDAR_CAM_EXTRIN ||
                                              task == CalibTaskType::CAM_CAM_EXTRIN ||
                                              task == CalibTaskType::LIDAR_LIDAR_EXTRIN ||
                                              task == CalibTaskType::IMU_LIDAR_EXTRIN);
            if (cfg_.allow_manual_fallback && task_supports_manual) {
                auto log_path_m = make_stage_log_path(CalibStage::MANUAL_REFINE, task);
                setup_stage_logger(log_path_m);
                if (r.needs_manual_refine())
                    UNICALIB_WARN("[Pipeline] 自动精标定 RMS={:.4f} 超过阈值, 进入手动校准",
                                  r.residual_rms);
                else
                    UNICALIB_INFO("[Pipeline] 用户已启用手动校准 (--manual), 进入 6-DOF 调整");
                log_stage_begin(CalibStage::MANUAL_REFINE, task,
                                "手动校准: 6-DOF 交互式调整");
                StageResult rm;
                try {
                    rm = run_manual_stage(task);
                } catch (const UniCalibException& e) {
                    rm = StageResult{};
                    rm.stage = CalibStage::MANUAL_REFINE;
                    rm.task = task;
                    rm.success = false;
                    rm.message = std::string("[") + errorCodeName(e.code()) + "] " + e.message();
                    rm.elapsed_ms = 0;
                    UNICALIB_ERROR("[Pipeline] 手动校准阶段异常: {}", rm.message);
                } catch (const std::exception& e) {
                    rm = StageResult{};
                    rm.stage = CalibStage::MANUAL_REFINE;
                    rm.task = task;
                    rm.success = false;
                    rm.message = std::string("std::exception: ") + e.what();
                    rm.elapsed_ms = 0;
                    UNICALIB_ERROR("[Pipeline] 手动校准阶段异常: {}", rm.message);
                } catch (...) {
                    rm = StageResult{};
                    rm.stage = CalibStage::MANUAL_REFINE;
                    rm.task = task;
                    rm.success = false;
                    rm.message = "未知异常";
                    rm.elapsed_ms = 0;
                    UNICALIB_ERROR("[Pipeline] 手动校准阶段未知异常");
                }
                rm.log_file = log_path_m;
                log_stage_end(rm);
                report_.stage_results.push_back(rm);
            }
        }
    }

    // 生成最终报告
    report_.final_params = params_;
    report_.print_summary();

    // 标定质量评估：检查各阶段残差是否超过阈值，超标时记录并建议手动
    for (const auto& sr : report_.stage_results) {
        if (sr.quality_threshold > 0 && sr.residual_rms > sr.quality_threshold) {
            UNICALIB_WARN("[质量评估] {} 残差 {:.4f} 超过阈值 {:.4f}，建议手动校准或重新采集数据",
                          task_str(sr.task), sr.residual_rms, sr.quality_threshold);
        }
    }

    std::string report_path = cfg_.output_dir + "/pipeline_report_" +
                               report_.pipeline_id.substr(9) + ".yaml";
    report_.save_report(report_path);

    // 将各阶段精度追加到对应 CSV，便于绘制曲线
    static const auto task_to_accuracy = [](CalibTaskType t) -> std::optional<CalibAccuracyTask> {
        switch (t) {
            case CalibTaskType::CAM_INTRINSIC:     return CalibAccuracyTask::CAM_INTRINSIC;
            case CalibTaskType::IMU_INTRINSIC:     return CalibAccuracyTask::IMU_INTRINSIC;
            case CalibTaskType::LIDAR_CAM_EXTRIN:  return CalibAccuracyTask::LIDAR_CAM_EXTRIN;
            case CalibTaskType::CAM_CAM_EXTRIN:    return CalibAccuracyTask::CAM_CAM_EXTRIN;
            case CalibTaskType::IMU_LIDAR_EXTRIN:   return CalibAccuracyTask::IMU_LIDAR_EXTRIN;
            default: return std::nullopt;
        }
    };
    for (const auto& r : report_.stage_results) {
        if (r.stage != CalibStage::FINE_AUTO && r.stage != CalibStage::MANUAL_REFINE)
            continue;
        auto at = task_to_accuracy(r.task);
        if (!at.has_value()) continue;
        append_stage_result_accuracy(cfg_.output_dir, *at,
            r.success, r.residual_rms, r.elapsed_ms, r.message);
    }

    return report_;
}

// ---------------------------------------------------------------------------
// 粗标定阶段入口 (占位实现 — 由子类或外部注入覆盖)
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_coarse_stage(CalibTaskType task) {
    UNICALIB_CALC("粗标定阶段开始 task={}", task_str(task));
    StageResult r;
    r.stage = CalibStage::COARSE_AI;
    r.task  = task;
    auto t_start = std::chrono::high_resolution_clock::now();

    // ─── LiDAR-Camera：必须经过粗标定；初值来源：(1) 主程序已跑 Stage1 并注入 (2) 配置 lidar_cam_coarse_initial (3) MIAS-LCEC ───
    if (task == CalibTaskType::LIDAR_CAM_EXTRIN) {
        if (coarse_lidar_cam_init_.has_value()) {
            r.success = true;
            r.residual_rms = 0.0;
            r.message = "使用主程序 Stage1 粗标定结果，作为精标定初值";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 使用主程序已注入的粗标定结果 t=[{:.3f},{:.3f},{:.3f}]m，精标定将以此起步",
                          coarse_lidar_cam_init_->translation().x(),
                          coarse_lidar_cam_init_->translation().y(),
                          coarse_lidar_cam_init_->translation().z());
            return r;
        }
        if (cfg_.lidar_cam_coarse_initial.has_value()) {
            coarse_lidar_cam_init_ = cfg_.lidar_cam_coarse_initial;
            r.success = true;
            r.residual_rms = 0.0;
            r.message = "使用配置 initial_extrinsic 作为粗标定结果，精标定将以此起步";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 使用配置粗标定初值作为粗标定结果 t=[{:.3f},{:.3f},{:.3f}]m，精标定将以此起步",
                          coarse_lidar_cam_init_->translation().x(),
                          coarse_lidar_cam_init_->translation().y(),
                          coarse_lidar_cam_init_->translation().z());
            return r;
        }
        coarse_lidar_cam_init_.reset();
        AICoarseCalibManager::Config ai_cfg;
        ai_cfg.ai_root    = cfg_.ai_models_root;
        ai_cfg.python_exe = cfg_.mias_lcec_python_exe.empty() ? cfg_.python_executable : cfg_.mias_lcec_python_exe;
        std::string repo_dir = cfg_.mias_lcec_repo_dir;
        if (!repo_dir.empty()) {
            fs::path rp(repo_dir);
            if (!rp.is_absolute() && !cfg_.ai_models_root.empty()) {
                std::string res = cfg_.ai_models_root;
                if (res.back() != '/' && repo_dir.front() != '/') res += "/";
                res += repo_dir;
                repo_dir = fs::path(res).lexically_normal().string();
            }
        }
        ai_cfg.mias_lcec.repo_dir       = repo_dir;
        ai_cfg.mias_lcec.calib_script   = cfg_.mias_lcec_calib_script.empty() ? "scripts/run_lidar_cam_coarse.py" : cfg_.mias_lcec_calib_script;
        if (!cfg_.mias_lcec_model_path.empty()) {
            fs::path mp(cfg_.mias_lcec_model_path);
            if (!mp.is_absolute() && !cfg_.ai_models_root.empty())
                ai_cfg.mias_lcec.model_path = (fs::path(cfg_.ai_models_root) / cfg_.mias_lcec_model_path).lexically_normal().string();
            else
                ai_cfg.mias_lcec.model_path = cfg_.mias_lcec_model_path;
        }
        ai_cfg.mias_lcec.allow_pnp_fallback = cfg_.mias_lcec_allow_pnp_fallback;
        ai_cfg.mias_lcec.timeout_sec    = cfg_.mias_lcec_timeout_sec > 0 ? cfg_.mias_lcec_timeout_sec : 180;
        ai_cfg.mias_lcec.work_dir       = cfg_.mias_lcec_work_dir.empty() ? "/tmp/mias_work" : cfg_.mias_lcec_work_dir;
        // 透传 LiDAR-Camera 粗标定(C3M) 与质量评估阈值
        ai_cfg.mias_lcec.c3m_use_iterative_refine = cfg_.lidar_cam_c3m_use_iterative_refine;
        ai_cfg.mias_lcec.c3m_iter_max = cfg_.lidar_cam_c3m_iter_max;
        ai_cfg.mias_lcec.c3m_iter_thresh = cfg_.lidar_cam_c3m_iter_thresh;
        ai_cfg.mias_lcec.c3m_similarity_threshold = cfg_.lidar_cam_c3m_similarity_threshold;
        ai_cfg.mias_lcec.quality_ncc_good = cfg_.lidar_cam_quality_ncc_good;
        ai_cfg.mias_lcec.quality_ncc_acceptable = cfg_.lidar_cam_quality_ncc_acceptable;
        ai_cfg.mias_lcec.quality_rms_good_px = cfg_.lidar_cam_quality_rms_good_px;
        ai_cfg.mias_lcec.quality_rms_acceptable_px = cfg_.lidar_cam_quality_rms_acceptable_px;
        ai_cfg.mias_lcec.quality_inlier_ratio_good = cfg_.lidar_cam_quality_inlier_ratio_good;
        ai_cfg.mias_lcec.quality_inlier_ratio_acceptable = cfg_.lidar_cam_quality_inlier_ratio_acceptable;
        std::string script_path = repo_dir.empty() ? "" : (repo_dir + "/" + ai_cfg.mias_lcec.calib_script);
        AICoarseCalibManager ai_mgr(ai_cfg);
        if (!ai_mgr.check_mias_lcec()) {
            r.success = false;
            r.message = "MIAS-LCEC 不可用 (repo/script 未配置或不存在)";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            UNICALIB_WARN("[Coarse-AI/LiDAR-Cam] {}", r.message);
            UNICALIB_WARN("[Coarse-AI/LiDAR-Cam] 已检查: repo_dir={}  script={}  (请确认 third_party.mias_lcec.repo_dir 与 ai_models_root，或提供 MIAS-LCEC 仓库及 scripts/run_lidar_cam_coarse.py)",
                          repo_dir.empty() ? "(未配置)" : repo_dir,
                          script_path.empty() ? "(未解析)" : script_path);
            return r;
        }
        std::string pcd_file, image_file;
        CameraIntrinsics cam_intrin;
        if (cfg_.use_ros2_bag && !cfg_.ros2_bag_file.empty()) {
            RosDataSourceConfig ros_cfg;
            ros_cfg.bag_file = cfg_.ros2_bag_file;
            ros_cfg.realtime_mode = false;
            ros_cfg.max_frames = 2;
            ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
            if (!cfg_.lidar_topics.empty()) {
                ros_cfg.lidar_topics = cfg_.lidar_topics;
                ros_cfg.lidar_ros2_topic = cfg_.lidar_topics.begin()->second;
            } else {
                ros_cfg.lidar_topics[cfg_.lidar_id] = cfg_.lidar_ros2_topic;
                ros_cfg.lidar_ros2_topic = cfg_.lidar_ros2_topic;
            }
            if (!cfg_.camera_topics.empty()) {
                ros_cfg.camera_topics = cfg_.camera_topics;
                ros_cfg.camera_ros2_topic = cfg_.camera_topics.begin()->second;
            } else {
                ros_cfg.camera_topics[cfg_.camera_id] = cfg_.camera_ros2_topic;
                ros_cfg.camera_ros2_topic = cfg_.camera_ros2_topic;
            }
            UnifiedDataLoader::Config load_cfg;
            load_cfg.source_type = UnifiedDataLoader::SourceType::ROS2_BAG;
            load_cfg.ros_config = ros_cfg;
            load_cfg.max_frames = 2;
            UnifiedDataLoader loader(load_cfg);
            if (loader.load()) {
                std::vector<LiDARScan> scans = loader.to_lidar_scans(cfg_.lidar_id);
                std::string first_cam = cfg_.lidar_camera_camera_list.empty() ? cfg_.camera_id : cfg_.lidar_camera_camera_list.front();
                std::vector<std::pair<double, cv::Mat>> frames = loader.to_camera_frames(first_cam);
                if (!scans.empty() && !frames.empty() && scans[0].cloud && !frames[0].second.empty()) {
                    std::string coarse_dir = cfg_.mias_lcec_work_dir + "/coarse_input";
                    fs::create_directories(coarse_dir);
                    pcd_file = coarse_dir + "/coarse_first.pcd";
                    image_file = coarse_dir + "/coarse_first.png";
                    if (pcl::io::savePCDFile(pcd_file, *scans[0].cloud, false) == 0 && cv::imwrite(image_file, frames[0].second)) {
                        cam_intrin.width  = frames[0].second.cols;
                        cam_intrin.height = frames[0].second.rows;
                        bool intrin_loaded = false;
                        std::string first_cam = cfg_.lidar_camera_camera_list.empty() ? cfg_.camera_id : cfg_.lidar_camera_camera_list.front();
                        auto it_cam_file = cfg_.camera_intrinsic_files.find(first_cam);
                        if (it_cam_file != cfg_.camera_intrinsic_files.end() && fs::exists(it_cam_file->second)) {
                            try {
                                cam_intrin = YamlIO::load_camera_intrinsics(it_cam_file->second);
                                intrin_loaded = (cam_intrin.width > 0 && cam_intrin.height > 0 && cam_intrin.fx > 0);
                            } catch (const std::exception&) {}
                        }
                        if (!intrin_loaded && !cfg_.camera_intrinsic_file.empty() && fs::exists(cfg_.camera_intrinsic_file)) {
                            try {
                                cam_intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
                                intrin_loaded = (cam_intrin.width > 0 && cam_intrin.height > 0 && cam_intrin.fx > 0);
                            } catch (const std::exception&) {}
                        }
                        if (!intrin_loaded) {
                            cam_intrin.fx = cam_intrin.fy = std::max(cam_intrin.width, cam_intrin.height) * 0.8;
                            cam_intrin.cx = cam_intrin.width * 0.5;
                            cam_intrin.cy = cam_intrin.height * 0.5;
                        }
                    } else {
                        pcd_file.clear();
                        image_file.clear();
                    }
                }
            }
        } else if (!cfg_.lidar_data_dir.empty()) {
            std::string lidar_dir = cfg_.lidar_data_dir;
            std::string img_dir = cfg_.camera_images_dir;
            if (!cfg_.lidar_camera_camera_list.empty() && cfg_.camera_images_dirs.count(cfg_.lidar_camera_camera_list.front()))
                img_dir = cfg_.camera_images_dirs.at(cfg_.lidar_camera_camera_list.front());
            else if (!cfg_.camera_id.empty() && cfg_.camera_images_dirs.count(cfg_.camera_id))
                img_dir = cfg_.camera_images_dirs.at(cfg_.camera_id);
            if (fs::exists(lidar_dir) && fs::exists(img_dir)) {
                std::vector<fs::path> pcd_paths, img_paths;
                for (const auto& e : fs::directory_iterator(lidar_dir)) {
                    std::string ext = e.path().extension().string();
                    if (ext == ".pcd" || ext == ".PCD") pcd_paths.push_back(e.path());
                }
                for (const auto& e : fs::directory_iterator(img_dir)) {
                    std::string ext = e.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp") img_paths.push_back(e.path());
                }
                std::sort(pcd_paths.begin(), pcd_paths.end());
                std::sort(img_paths.begin(), img_paths.end());
                if (!pcd_paths.empty() && !img_paths.empty()) {
                    pcd_file = pcd_paths.front().string();
                    image_file = img_paths.front().string();
                    cv::Mat img = cv::imread(image_file);
                    if (!img.empty()) {
                        cam_intrin.width = img.cols;
                        cam_intrin.height = img.rows;
                        bool intrin_loaded = false;
                        std::string first_cam = cfg_.lidar_camera_camera_list.empty() ? cfg_.camera_id : cfg_.lidar_camera_camera_list.front();
                        auto it_cam_file = cfg_.camera_intrinsic_files.find(first_cam);
                        if (it_cam_file != cfg_.camera_intrinsic_files.end() && fs::exists(it_cam_file->second)) {
                            try {
                                cam_intrin = YamlIO::load_camera_intrinsics(it_cam_file->second);
                                intrin_loaded = (cam_intrin.width > 0 && cam_intrin.height > 0 && cam_intrin.fx > 0);
                            } catch (const std::exception&) {}
                        }
                        if (!intrin_loaded && !cfg_.camera_intrinsic_file.empty() && fs::exists(cfg_.camera_intrinsic_file)) {
                            try {
                                cam_intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
                                intrin_loaded = (cam_intrin.width > 0 && cam_intrin.height > 0 && cam_intrin.fx > 0);
                            } catch (const std::exception&) {}
                        }
                        if (!intrin_loaded) {
                            cam_intrin.fx = cam_intrin.fy = std::max(cam_intrin.width, cam_intrin.height) * 0.8;
                            cam_intrin.cx = cam_intrin.width * 0.5;
                            cam_intrin.cy = cam_intrin.height * 0.5;
                        }
                    }
                }
            }
        }
        if (!pcd_file.empty() && !image_file.empty()) {
            UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 输入摘要  pcd={}  image={}  内参 fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}  size={}x{}",
                          pcd_file, image_file, cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy, cam_intrin.width, cam_intrin.height);
            auto coarse_result = ai_mgr.coarse_lidar_cam(pcd_file, image_file, cam_intrin, cfg_.lidar_id, cfg_.camera_id);
            if (coarse_result.has_value()) {
                coarse_lidar_cam_init_ = coarse_result->SE3_TargetInRef();
                r.success = true;
                r.residual_rms = 0.0;
                r.message = "MIAS-LCEC 粗标定成功，初值将用于精标定";
                UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 粗标定成功 t=[{:.3f},{:.3f},{:.3f}]m",
                              coarse_lidar_cam_init_->translation().x(),
                              coarse_lidar_cam_init_->translation().y(),
                              coarse_lidar_cam_init_->translation().z());
            } else {
                if (cfg_.lidar_cam_coarse_initial.has_value()) {
                    coarse_lidar_cam_init_ = cfg_.lidar_cam_coarse_initial;
                    r.success = true;
                    r.message = "MIAS-LCEC 未成功，使用配置粗标定初值作为粗标定结果，精标定将以此起步";
                    UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 使用配置初值作为粗标定结果 t=[{:.3f},{:.3f},{:.3f}]m",
                                  coarse_lidar_cam_init_->translation().x(),
                                  coarse_lidar_cam_init_->translation().y(),
                                  coarse_lidar_cam_init_->translation().z());
                } else {
                    r.success = false;
                    r.message = "AI粗标定失败，精标定将使用 identity 初值";
                    UNICALIB_WARN("[Coarse-AI/LiDAR-Cam] {}", r.message);
                }
            }
        } else {
            if (cfg_.lidar_cam_coarse_initial.has_value()) {
                coarse_lidar_cam_init_ = cfg_.lidar_cam_coarse_initial;
                r.success = true;
                r.message = "粗标定无输入，使用配置初值作为粗标定结果，精标定将以此起步";
                UNICALIB_INFO("[Coarse-AI/LiDAR-Cam] 使用配置初值 t=[{:.3f},{:.3f},{:.3f}]m", coarse_lidar_cam_init_->translation().x(), coarse_lidar_cam_init_->translation().y(), coarse_lidar_cam_init_->translation().z());
            } else {
                r.success = false;
                r.message = "粗标定无输入 (未配置数据或首帧不可用)，精标定将使用 identity";
                UNICALIB_WARN("[Coarse-AI/LiDAR-Cam] {}  pcd_empty={}  image_empty={}", r.message, pcd_file.empty(), image_file.empty());
            }
        }
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        return r;
    }

    UNICALIB_INFO("[Coarse-AI] 任务: {}", task_str(task));
    UNICALIB_INFO("[Coarse-AI] 说明: 此阶段需通过 AICoarseCalibManager 注入");
    UNICALIB_INFO("[Coarse-AI] 模型映射:");
    switch (task) {
        case CalibTaskType::CAM_INTRINSIC:
            UNICALIB_INFO("  → DM-Calib (扩散模型单图内参估计)");
            UNICALIB_INFO("  调用: {}/DM-Calib/DMCalib/tools/infer_unicalib.py",
                          cfg_.ai_models_root);
            break;
        case CalibTaskType::IMU_INTRINSIC:
            UNICALIB_INFO("  → Transformer-IMU-Calibrator");
            UNICALIB_INFO("  调用: {}/Transformer-IMU-Calibrator/eval.py",
                          cfg_.ai_models_root);
            break;
        case CalibTaskType::IMU_LIDAR_EXTRIN:
            UNICALIB_INFO("  → L2Calib (RL强化学习 SE(3)-流形)");
            UNICALIB_INFO("  调用: {}/learn-to-calibrate/train.py",
                          cfg_.ai_models_root);
            break;
        case CalibTaskType::LIDAR_CAM_EXTRIN: {
            std::string repo = cfg_.mias_lcec_repo_dir.empty() ?
                (cfg_.ai_models_root + "/MIAS-LCEC") : cfg_.mias_lcec_repo_dir;
            UNICALIB_INFO("  → MIAS-LCEC  script={}", cfg_.mias_lcec_calib_script);
            UNICALIB_INFO("  调用: {}", repo);
            break;
        }
        case CalibTaskType::CAM_CAM_EXTRIN:
            UNICALIB_INFO("  → 特征匹配初始化 (ORB/SIFT + 本质矩阵)");
            break;
        default: break;
    }

    r.success  = true;
    r.message  = "AI粗标定占位 — 请通过 AICoarseCalibManager 注入实际调用";
    r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
    return r;
}

// ---------------------------------------------------------------------------
// 精标定阶段入口
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_stage(CalibTaskType task) {
    UNICALIB_CALC("精标定阶段开始 task={}", task_str(task));
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = task;

    UNICALIB_INFO("[Fine-Auto] 任务: {} | 无目标优先: {}",
                  task_str(task), cfg_.prefer_targetfree);

    // 设置质量阈值
    switch (task) {
        case CalibTaskType::LIDAR_CAM_EXTRIN:
            r.quality_threshold = cfg_.lidar_cam_rms_threshold;
            UNICALIB_INFO("[Fine-Auto] 方法: {}",
                          cfg_.prefer_targetfree ?
                          "边缘对齐互信息 (EDGE_ALIGNMENT, 无目标)" :
                          "棋盘格 PnP (TARGET_CHESSBOARD)");
            break;
        case CalibTaskType::CAM_CAM_EXTRIN:
            r.quality_threshold = cfg_.cam_cam_rms_threshold;
            UNICALIB_INFO("[Fine-Auto] 方法: {}",
                          cfg_.prefer_targetfree ?
                          "特征匹配 + Bundle Adjustment (无目标)" :
                          "OpenCV stereoCalibrate (棋盘格)");
            break;
        case CalibTaskType::IMU_LIDAR_EXTRIN:
            r.quality_threshold = cfg_.imu_lidar_rot_threshold;
            UNICALIB_INFO("[Fine-Auto] 方法: B样条连续时间优化 (无目标)");
            break;
        case CalibTaskType::LIDAR_LIDAR_EXTRIN:
            UNICALIB_INFO("[Fine-Auto] 方法: 请使用 unicalib_lidar_lidar 独立应用进行 LiDAR-LiDAR 标定");
            break;
        case CalibTaskType::IMU_INTRINSIC:
            UNICALIB_INFO("[Fine-Auto] 方法: Allan方差分析");
            break;
        case CalibTaskType::CAM_INTRINSIC:
            UNICALIB_INFO("[Fine-Auto] 方法: {}",
                          cfg_.prefer_targetfree ?
                          "DM-Calib精化 + RANSAC" :
                          "棋盘格角点检测");
            break;
        default: break;
    }

    auto t_start = std::chrono::high_resolution_clock::now();

    // ─── LiDAR-Camera 外参精标定 ─────────────────────────────────────────
    if (task == CalibTaskType::LIDAR_CAM_EXTRIN) {
        return run_fine_lidar_camera();
    }

    // ─── IMU-LiDAR 外参精标定（支持 imu_lidar.pairs 多对）────────────────
    if (task == CalibTaskType::IMU_LIDAR_EXTRIN) {
        return run_fine_imu_lidar();
    }

    // ─── IMU 内参精标定 ─────────────────────────────────────────────────
    if (task == CalibTaskType::IMU_INTRINSIC) {
        return run_fine_imu_intrinsic();
    }

    // ─── Camera-Camera 外参精标定（多相机按数目自动两两标定）──────────────
    if (task == CalibTaskType::CAM_CAM_EXTRIN) {
        return run_fine_cam_cam();
    }

    // ─── LiDAR-LiDAR 外参：由独立应用 unicalib_lidar_lidar 执行 ─────────────
    if (task == CalibTaskType::LIDAR_LIDAR_EXTRIN) {
        r.success = false;
        r.message = "LiDAR-LiDAR 标定请使用: unicalib_lidar_lidar --config <config.yaml>";
        auto t_end = std::chrono::high_resolution_clock::now();
        r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        return r;
    }

    // ─── 其他任务（占位）─────────────────────────────────────────────────
    r.success  = true;
    r.message  = "精标定占位 — 请通过具体标定器实现";
    auto t_end = std::chrono::high_resolution_clock::now();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    UNICALIB_WARN("[Fine-Auto] 任务 {} 当前为占位实现", task_str(task));
    return r;
}

// ---------------------------------------------------------------------------
// LiDAR-Camera 精标定实现
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_lidar_camera() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::LIDAR_CAM_EXTRIN;
    r.quality_threshold = cfg_.lidar_cam_rms_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    // ─── 数据源类型判断 ─────────────────────────────────────────────────
    enum class DataSourceType {
        FILES,
        ROS2_BAG,
        ROS2_TOPIC,
        NEW_FORMAT
    };
    
    DataSourceType data_source_type = DataSourceType::FILES;
    if (cfg_.use_new_format) {
        data_source_type = DataSourceType::NEW_FORMAT;
    } else if (cfg_.use_ros2_bag && !cfg_.ros2_bag_file.empty()) {
        data_source_type = DataSourceType::ROS2_BAG;
    } else if (cfg_.use_ros2_topics && (!cfg_.lidar_ros2_topic.empty() || !cfg_.camera_ros2_topic.empty())) {
        data_source_type = DataSourceType::ROS2_TOPIC;
    }

    // ─── 显示数据源信息 ───────────────────────────────────────────────
    UNICALIB_INFO("[STAGE=lidar_cam_fine_auto] LiDAR-Camera 精标定 开始");
    UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 开始执行精标定...");
    const char* data_src_str = (data_source_type == DataSourceType::ROS2_BAG) ? "ROS2_BAG" :
                               (data_source_type == DataSourceType::ROS2_TOPIC) ? "ROS2_TOPIC" :
                               (data_source_type == DataSourceType::NEW_FORMAT) ? "NEW_FORMAT" : "FILES";
    UNICALIB_INFO("  [精标定数据源] 类型={}  lidar_id={}", data_src_str, cfg_.lidar_id);

    if (data_source_type == DataSourceType::ROS2_BAG) {
        UNICALIB_INFO("  ROS2 Bag: {}  LiDAR 话题: {}  相机话题: {}",
                      cfg_.ros2_bag_file, cfg_.lidar_ros2_topic, cfg_.camera_ros2_topic);
    } else if (data_source_type == DataSourceType::ROS2_TOPIC) {
        UNICALIB_INFO("  ROS2 实时: LiDAR={}  相机={}  最大等待={:.1f}s",
                      cfg_.lidar_ros2_topic, cfg_.camera_ros2_topic, cfg_.ros2_max_wait_time);
    } else if (data_source_type == DataSourceType::NEW_FORMAT) {
        UNICALIB_INFO("  新格式目录: {}  时间戳单位={}",
                      cfg_.new_format_root_dir.empty() ? "(未设置)" : cfg_.new_format_root_dir,
                      cfg_.new_format_timestamp_unit);
    } else {
        UNICALIB_INFO("  文件目录: lidar={}  camera={}",
                      cfg_.lidar_data_dir.empty() ? "(未设置)" : cfg_.lidar_data_dir,
                      cfg_.camera_images_dir.empty() ? "(未设置)" : cfg_.camera_images_dir);
    }

    // 待标定相机列表：优先 lidar_camera.pairs（显式 [lidar_id, cam_id]），否则 camera_list 或 camera_id
    std::vector<std::string> cameras_to_run;
    if (!cfg_.lidar_camera_pairs.empty()) {
        for (const auto& [lidar_id, cam_id] : cfg_.lidar_camera_pairs) {
            if (lidar_id == cfg_.lidar_id)
                cameras_to_run.push_back(cam_id);
        }
        std::sort(cameras_to_run.begin(), cameras_to_run.end());
        cameras_to_run.erase(std::unique(cameras_to_run.begin(), cameras_to_run.end()), cameras_to_run.end());
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 使用 lidar_camera.pairs：当前 lidar={} 下 {} 个相机", cfg_.lidar_id, cameras_to_run.size());
    }
    if (cameras_to_run.empty()) {
        cameras_to_run = cfg_.lidar_camera_camera_list.empty()
            ? std::vector<std::string>{cfg_.camera_id}
            : cfg_.lidar_camera_camera_list;
    }

    // ─── 1. 数据路径/配置验证 ───────────────────────────────────────
    if (data_source_type == DataSourceType::FILES) {
        if (cfg_.lidar_data_dir.empty()) {
            r.success = false;
            r.message = "数据路径未配置 — 请在 PipelineConfig 中设置 lidar_data_dir";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
        for (const std::string& cam : cameras_to_run) {
            std::string img_dir = cfg_.camera_images_dirs.count(cam) ? cfg_.camera_images_dirs.at(cam) : cfg_.camera_images_dir;
            if (img_dir.empty() || !fs::exists(img_dir)) {
                r.success = false;
                r.message = "相机图像目录未配置或不存在 (camera=" + cam + "): " + img_dir;
                auto t_end = std::chrono::high_resolution_clock::now();
                r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
                UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
                return r;
            }
        }
        if (!fs::exists(cfg_.lidar_data_dir)) {
            r.success = false;
            r.message = "LiDAR 数据目录不存在: " + cfg_.lidar_data_dir;
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    } else if (data_source_type == DataSourceType::ROS2_BAG) {
        if (!fs::exists(cfg_.ros2_bag_file)) {
            r.success = false;
            r.message = "ROS2 Bag 路径不存在: " + cfg_.ros2_bag_file + "（可为目录，rosbag2 格式）";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    } else if (data_source_type == DataSourceType::ROS2_TOPIC) {
        if (cfg_.lidar_ros2_topic.empty() || cfg_.camera_ros2_topic.empty()) {
            r.success = false;
            r.message = "ROS2 实时模式需同时配置 LiDAR 与相机话题（config 中 ros2.lidar_topic / ros2.camera_topic 或 sensors[].topic）";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    } else if (data_source_type == DataSourceType::NEW_FORMAT) {
        const std::string nf_root = resolve_new_format_root_dir(cfg_.new_format_root_dir);
        if (nf_root.empty()) {
            r.success = false;
            r.message = "NEW_FORMAT 模式需配置 new_format.root_dir";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
        if (!fs::exists(nf_root)) {
            r.success = false;
            r.message = "NEW_FORMAT 根目录不存在: " + nf_root +
                        (nf_root == cfg_.new_format_root_dir ? "" : (" (配置中为: " + cfg_.new_format_root_dir + ")"));
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    }

    // ─── 2. 加载 LiDAR 数据 (一次)，ROS2 时保留 loader 供多相机用 ─────────
    std::vector<LiDARScan> lidar_scans;
    std::optional<UnifiedDataLoader> loader_opt;

    if (data_source_type == DataSourceType::FILES) {
        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 从文件加载点云");

        // 检查目录是否存在
        if (!fs::exists(cfg_.lidar_data_dir) || !fs::is_directory(cfg_.lidar_data_dir)) {
            r.success = false;
            r.message = "LiDAR 数据目录不存在或不是目录: " + cfg_.lidar_data_dir;
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }

        std::vector<fs::path> pcd_files;
        try {
            for (const auto& entry : fs::directory_iterator(cfg_.lidar_data_dir)) {
                if (entry.is_regular_file() &&
                    (entry.path().extension() == ".pcd" || entry.path().extension() == ".PCD")) {
                    pcd_files.push_back(entry.path());
                }
            }
        } catch (const fs::filesystem_error& e) {
            r.success = false;
            r.message = std::string("读取 LiDAR 目录失败: ") + e.what();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }

        std::sort(pcd_files.begin(), pcd_files.end());
        if (pcd_files.empty()) {
            r.success = false;
            r.message = "未找到 PCD 文件: " + cfg_.lidar_data_dir;
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        const size_t MAX_FRAMES = 100;
        size_t step = std::max(size_t(1), pcd_files.size() / MAX_FRAMES);
        bool lidar_ts_fallback_used = false;
        for (size_t i = 0; i < pcd_files.size() && lidar_scans.size() < MAX_FRAMES; i += step) {
            LiDARScan scan;
            scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files[i].string(), *scan.cloud) == 0) {
                std::string fname = pcd_files[i].stem().string();
                try { scan.timestamp = std::stod(fname); } catch (...) {
                    scan.timestamp = static_cast<double>(i) * 0.1;
                    lidar_ts_fallback_used = true;
                }
                lidar_scans.push_back(std::move(scan));
            }
        }
        if (lidar_ts_fallback_used)
            UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 部分/全部 PCD 时间戳由文件名解析失败，已回退为索引*0.1（按索引与图像对齐）；若为 SLAM 数据请确认帧率与顺序一致");
        if (lidar_scans.empty()) {
            r.success = false;
            r.message = "未能成功加载任何点云";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "点云加载完成 — {} 帧", lidar_scans.size());
    } else if (data_source_type == DataSourceType::ROS2_BAG || data_source_type == DataSourceType::ROS2_TOPIC) {
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 从 ROS2 加载数据...");
        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.realtime_mode = (data_source_type == DataSourceType::ROS2_TOPIC);
        ros_cfg.realtime_timeout = cfg_.ros2_max_wait_time;
        ros_cfg.sample_interval = cfg_.ros2_sample_interval;
        ros_cfg.max_frames = cfg_.ros2_max_frames;
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        if (!cfg_.lidar_topics.empty()) {
            ros_cfg.lidar_topics = cfg_.lidar_topics;
            ros_cfg.lidar_ros2_topic = cfg_.lidar_topics.begin()->second;
        } else if (!cfg_.lidar_ros2_topic.empty()) {
            ros_cfg.lidar_topics[cfg_.lidar_id] = cfg_.lidar_ros2_topic;
            ros_cfg.lidar_ros2_topic = cfg_.lidar_ros2_topic;
        }
        if (!cfg_.camera_topics.empty()) {
            ros_cfg.camera_topics = cfg_.camera_topics;
            ros_cfg.camera_ros2_topic = cfg_.camera_topics.begin()->second;
        } else if (!cfg_.camera_ros2_topic.empty()) {
            ros_cfg.camera_topics[cfg_.camera_id] = cfg_.camera_ros2_topic;
            ros_cfg.camera_ros2_topic = cfg_.camera_ros2_topic;
        }
        if (!cfg_.imu_topics.empty()) {
            ros_cfg.imu_topics = cfg_.imu_topics;
            ros_cfg.imu_ros2_topic = cfg_.imu_topics.begin()->second;
        } else if (!cfg_.imu_ros2_topic.empty()) {
            ros_cfg.imu_topics["imu_0"] = cfg_.imu_ros2_topic;
            ros_cfg.imu_ros2_topic = cfg_.imu_ros2_topic;
        }
        UnifiedDataLoader::Config unified_cfg;
        unified_cfg.source_type = (data_source_type == DataSourceType::ROS2_BAG) ?
            UnifiedDataLoader::SourceType::ROS2_BAG : UnifiedDataLoader::SourceType::ROS2_TOPIC;
        unified_cfg.ros_config = ros_cfg;
        unified_cfg.max_frames = cfg_.ros2_max_frames;
        loader_opt.emplace(unified_cfg);
        if (!loader_opt->load()) {
            r.success = false;
            r.message = "ROS2 数据加载失败: " + loader_opt->get_status_message();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        lidar_scans = loader_opt->to_lidar_scans(cfg_.lidar_id);
        if (lidar_scans.empty()) {
            r.success = false;
            r.message = "未能从 ROS2 加载 LiDAR 数据";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] LiDAR 加载完成: {} 帧 (将按相机逐路取帧)", lidar_scans.size());
    } else if (data_source_type == DataSourceType::NEW_FORMAT) {
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 从 NEW_FORMAT 加载数据...");
        UnifiedDataLoader::Config unified_cfg;
        unified_cfg.source_type = UnifiedDataLoader::SourceType::NEW_FORMAT;
        unified_cfg.new_format_root_dir = resolve_new_format_root_dir(cfg_.new_format_root_dir);
        unified_cfg.new_format_lidar_index_files = cfg_.new_format_lidar_index_files;
        unified_cfg.new_format_camera_index_files = cfg_.new_format_camera_index_files;
        unified_cfg.new_format_imu_index_files = cfg_.new_format_imu_index_files;
        unified_cfg.new_format_timestamp_unit = cfg_.new_format_timestamp_unit;
        unified_cfg.new_format_oem7_imu_rate_hz = cfg_.new_format_oem7_imu_rate_hz;
        unified_cfg.new_format_oem7_time_base = cfg_.new_format_oem7_time_base;
        unified_cfg.new_format_oem7_gps_utc_leap_sec = cfg_.new_format_oem7_gps_utc_leap_sec;
        unified_cfg.new_format_oem7_time_offset_sec = cfg_.new_format_oem7_time_offset_sec;
        unified_cfg.max_frames = cfg_.ros2_max_frames;
        unified_cfg.sample_interval = cfg_.ros2_sample_interval;
        loader_opt.emplace(unified_cfg);
        if (!loader_opt->load()) {
            r.success = false;
            r.message = "NEW_FORMAT 数据加载失败: " + loader_opt->get_status_message();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        lidar_scans = loader_opt->to_lidar_scans(cfg_.lidar_id);
        if (lidar_scans.empty()) {
            r.success = false;
            r.message = "未能从 NEW_FORMAT 加载 LiDAR 数据（请检查 lidar 索引和 sensor_id）";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] LiDAR 加载完成: {} 帧 (NEW_FORMAT)", lidar_scans.size());
    }

    const size_t MAX_FRAMES = 100;
    int cameras_done = 0;
    double max_rms = 0.0;
    std::string result_subdir = cfg_.output_dir;
    if (cameras_to_run.size() > 1) {
        result_subdir = cfg_.output_dir + "/lidar_camera_extrinsic";
        std::error_code ec;
        fs::create_directories(result_subdir, ec);
    }

    for (const std::string& cur_cam : cameras_to_run) {
#if UNICALIB_WITH_PANGOLIN
        CalibVisualizer::Ptr lidar_cam_viz;
#endif
        std::vector<std::pair<double, cv::Mat>> camera_frames;
        if (data_source_type == DataSourceType::FILES) {
            std::string img_dir = cfg_.camera_images_dirs.count(cur_cam) ? cfg_.camera_images_dirs.at(cur_cam) : cfg_.camera_images_dir;

            // 检查图像目录是否存在
            if (!fs::exists(img_dir) || !fs::is_directory(img_dir)) {
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 相机 {} 图像目录不存在或无效: {}", cur_cam, img_dir);
                continue;
            }

            std::vector<fs::path> img_files;
            try {
                for (const auto& entry : fs::directory_iterator(img_dir)) {
                    if (!entry.is_regular_file()) continue;
                    std::string ext = entry.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp")
                        img_files.push_back(entry.path());
                }
            } catch (const fs::filesystem_error& e) {
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 读取相机 {} 目录失败: {}", cur_cam, e.what());
                continue;
            }

            std::sort(img_files.begin(), img_files.end());
            if (img_files.empty()) {
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 相机 {} 无图像: {}", cur_cam, img_dir);
                continue;
            }
            size_t img_step = std::max(size_t(1), img_files.size() / MAX_FRAMES);
            bool cam_ts_fallback_used = false;
            for (size_t i = 0; i < img_files.size() && camera_frames.size() < MAX_FRAMES; i += img_step) {
                cv::Mat img = cv::imread(img_files[i].string());
                if (!img.empty()) {
                    std::string fname = img_files[i].stem().string();
                    double ts;
                    try { ts = std::stod(fname); } catch (...) {
                        ts = static_cast<double>(i) * 0.1;
                        cam_ts_fallback_used = true;
                    }
                    camera_frames.emplace_back(ts, img);
                }
            }
            if (cam_ts_fallback_used)
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 相机 {} 部分/全部图像时间戳由文件名解析失败，已回退为索引*0.1；若为 SLAM 数据请确认与 LiDAR 帧率及顺序一致", cur_cam);
        } else {
            camera_frames = loader_opt->to_camera_frames(cur_cam);
        }
        if (camera_frames.empty()) {
            UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 相机 {} 无帧，跳过", cur_cam);
            continue;
        }

        CameraIntrinsics cam_intrin;
        auto it_file = cfg_.camera_intrinsic_files.find(cur_cam);
        if (it_file != cfg_.camera_intrinsic_files.end() && fs::exists(it_file->second)) {
            try {
                cam_intrin = YamlIO::load_camera_intrinsics(it_file->second);
            } catch (...) {
                cam_intrin = infer_intrinsics_from_images(camera_frames);
            }
        } else if (params_ && params_->camera_intrinsics.count(cur_cam) && params_->camera_intrinsics.at(cur_cam)) {
            cam_intrin = *params_->camera_intrinsics.at(cur_cam);
        } else if (!cfg_.camera_intrinsic_file.empty() && fs::exists(cfg_.camera_intrinsic_file)) {
            try {
                cam_intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
            } catch (...) {
                cam_intrin = infer_intrinsics_from_images(camera_frames);
            }
        } else {
            cam_intrin = infer_intrinsics_from_images(camera_frames);
        }

        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: LiDAR-Camera 标定 相机={} (方法: {})",
                          cur_cam, cfg_.prefer_targetfree ? "边缘对齐" : "棋盘格");
        LiDARCameraCalibrator::Config calib_cfg;
        if (cfg_.lidar_cam_method == "edge") {
            calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
        } else if (cfg_.lidar_cam_method == "target") {
            calib_cfg.method = LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
        } else if (cfg_.lidar_cam_method == "motion") {
            calib_cfg.method = LiDARCameraCalibrator::Method::MOTION_BSPLINE;
        } else {
            calib_cfg.method = cfg_.prefer_targetfree ?
                LiDARCameraCalibrator::Method::EDGE_ALIGNMENT :
                LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
        }
        calib_cfg.board_cols     = cfg_.board_cols;
        calib_cfg.board_rows     = cfg_.board_rows;
        calib_cfg.square_size_m  = cfg_.square_size_m;
        if (cfg_.lidar_cam_target_type == "circles_grid")
            calib_cfg.target_type = LiDARCameraCalibrator::TargetType::CIRCLES_GRID;
        else if (cfg_.lidar_cam_target_type == "asym_circles")
            calib_cfg.target_type = LiDARCameraCalibrator::TargetType::ASYMMETRIC_CIRCLES;
        else
            calib_cfg.target_type = LiDARCameraCalibrator::TargetType::CHESSBOARD;
        calib_cfg.target_min_corners_per_frame = cfg_.target_min_corners_per_frame;
        calib_cfg.target_min_frames = cfg_.target_min_frames;
        calib_cfg.target_use_coarse_rotation_search = cfg_.target_use_coarse_rotation_search;
        calib_cfg.target_coarse_rotation_range_deg = cfg_.target_coarse_rotation_range_deg;
        calib_cfg.target_coarse_rotation_step_deg = cfg_.target_coarse_rotation_step_deg;
        calib_cfg.target_coarse_inlier_threshold_px = cfg_.target_coarse_inlier_threshold_px;
        calib_cfg.target_per_frame_rms_threshold_px = cfg_.target_per_frame_rms_threshold_px;
        calib_cfg.target_ba_max_iter = cfg_.target_ba_max_iter;
        calib_cfg.target_ba_use_robust_loss = cfg_.target_ba_use_robust_loss;
        calib_cfg.target_ba_huber_scale_px = cfg_.target_ba_huber_scale_px;
        calib_cfg.edge_canny_low  = cfg_.edge_canny_low;
        calib_cfg.edge_canny_high = cfg_.edge_canny_high;
        calib_cfg.ceres_max_iter  = cfg_.ceres_max_iter;
        calib_cfg.frame_sync_threshold_s = cfg_.frame_sync_threshold_s;
        calib_cfg.time_offset_search_range_s = cfg_.time_offset_search_range_s;
        calib_cfg.ncc_threshold = cfg_.ncc_threshold;
        calib_cfg.ncc_low_skip_threshold  = cfg_.ncc_low_skip_threshold;
        calib_cfg.optimize_time_offset = cfg_.lidar_cam_optimize_time_offset;
        calib_cfg.edge_weight   = cfg_.lidar_cam_edge_weight;
        calib_cfg.corner_weight = cfg_.lidar_cam_corner_weight;
        calib_cfg.intensity_weight = cfg_.lidar_cam_intensity_weight;
        calib_cfg.corner_max_per_frame = cfg_.lidar_cam_corner_max_per_frame;
        calib_cfg.use_robust_loss = cfg_.lidar_cam_use_robust_loss;
        calib_cfg.robust_loss_type = cfg_.lidar_cam_robust_loss_type;
        calib_cfg.robust_loss_threshold = cfg_.lidar_cam_robust_loss_threshold;
        calib_cfg.enable_motion_compensation = cfg_.lidar_cam_motion_compensation_enable;
        calib_cfg.motion_compensation_method = cfg_.lidar_cam_motion_compensation_method;
        calib_cfg.verbose = (cfg_.log_level == "debug" || cfg_.log_level == "trace");

        LiDARCameraCalibrator calibrator(calib_cfg);
        calibrator.set_progress_callback([this](const std::string& step, double prog) {
            if (prog >= 0)
                log_step(CalibStage::FINE_AUTO, step, "进度 " + std::to_string(static_cast<int>(prog * 100)) + "%");
        });

#if UNICALIB_WITH_PANGOLIN
        if (cfg_.enable_viz) {
            try {
                lidar_cam_viz = CalibVisualizer::Create();
                calibrator.set_visualizer(lidar_cam_viz);
                calibrator.enable_realtime_viz(true);
                UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 可视化已启用：仅标定结束后显示一帧「图像+点云投影」叠加，不创建 3D 窗口、不泵送其他帧");
            } catch (const std::exception& e) {
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 可视化启动失败（继续无界面运行）: {}", e.what());
            }
        }
#endif

        // 初值优先级：
        // 1) lidar_camera.initial_extrinsics[`${lidar}__${camera}`]（或带 T_ 前缀）
        // 2) 全局 coarse_lidar_cam_init_（来自 initial_extrinsic/T_cam_lidar 或 Stage-1 粗标定）
        std::optional<Sophus::SE3d> coarse_init;
        const std::string key = cfg_.lidar_id + "__" + cur_cam;
        const std::string key_alt = "T_" + cfg_.lidar_id + "__" + cur_cam;
        auto it_inline = cfg_.lidar_camera_initial_extrinsic_inline.find(key);
        if (it_inline == cfg_.lidar_camera_initial_extrinsic_inline.end())
            it_inline = cfg_.lidar_camera_initial_extrinsic_inline.find(key_alt);
        if (it_inline != cfg_.lidar_camera_initial_extrinsic_inline.end() && it_inline->second.size() >= 16u) {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 16; ++i) T(i / 4, i % 4) = it_inline->second[static_cast<size_t>(i)];
            coarse_init = Sophus::SE3d(T);
            UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 使用按相机初值 key={} (camera={})", it_inline->first, cur_cam);
        } else {
            coarse_init = coarse_lidar_cam_init_;
        }
        if (coarse_init.has_value()) {
            UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 使用粗标定初值 t=[{:.3f},{:.3f},{:.3f}]m",
                          coarse_init->translation().x(), coarse_init->translation().y(), coarse_init->translation().z());
        }

        if (cfg_.lidar_cam_use_config_extrinsic_only && coarse_init.has_value()) {
            // 仅用配置初值：跳过精标定优化，直接以配置外参作为结果，供手动微调
            LiDARCameraCalibrator::TwoStageResult result;
            ExtrinsicSE3 ext;
            ext.ref_sensor_id = cfg_.lidar_id;
            ext.target_sensor_id = cur_cam;
            ext.set_SE3(*coarse_init);
            ext.residual_rms = 0.0;
            ext.is_converged = false;
            result.coarse = ext;
            result.coarse_rms = 0.0;
            result.coarse_method = "config_initial";
            result.fine = ext;
            result.fine_rms = 0.0;
            result.fine_method = "config_initial_skip_fine";
            if (params_) {
                auto extrin_ptr = params_->get_or_create_extrinsic(cfg_.lidar_id, cur_cam);
                if (extrin_ptr) *extrin_ptr = ext;
            }
            if (!manual_cache_.lidar_scan.has_value() && !lidar_scans.empty() && !camera_frames.empty()) {
                manual_cache_.lidar_scan = lidar_scans[0];
                manual_cache_.camera_image = camera_frames[0].second.clone();
                manual_cache_.lidar_id = cfg_.lidar_id;
                manual_cache_.camera_id = cur_cam;
                manual_cache_.camera_intrin = cam_intrin;
            }
            std::string result_yaml = result_subdir + "/lidar_cam_" + cur_cam + "_extrinsic.yaml";
            save_extrinsic_result(result_yaml, result, cur_cam);
            if (!lidar_scans.empty() && !camera_frames.empty()) {
                std::string vis_path = result_subdir + "/lidar_cam_" + cur_cam + "_projection.png";
                UNICALIB_INFO("[Viz] 即将生成投影图并保存到 {}", vis_path);
                calibrator.visualize_projection(
                    lidar_scans[0], camera_frames[0].second, ext, cam_intrin, vis_path);
                UNICALIB_INFO("[Viz] 投影图已保存（仅使用配置初值，未做精标定）");
            }
            max_rms = std::max(max_rms, 0.0);
            cameras_done++;
            UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 相机 {} 已使用配置初值跳过精标定，直接供手动微调", cur_cam);
        } else {
            UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 执行标定 相机={}: 点云 {} 帧  图像 {} 帧",
                          cur_cam, lidar_scans.size(), camera_frames.size());
            auto result = calibrator.calibrate_two_stage(
                lidar_scans, camera_frames, cam_intrin,
                coarse_init, cfg_.prefer_targetfree,
                cfg_.lidar_id, cur_cam);

            if (result.best() != nullptr) {
                if (result.fine.has_value() && params_) {
                    auto extrin_ptr = params_->get_or_create_extrinsic(cfg_.lidar_id, cur_cam);
                    if (extrin_ptr) *extrin_ptr = *result.fine;
                }
                if (!manual_cache_.lidar_scan.has_value() && !lidar_scans.empty() && !camera_frames.empty()) {
                    manual_cache_.lidar_scan = lidar_scans[0];
                    manual_cache_.camera_image = camera_frames[0].second.clone();
                    manual_cache_.lidar_id = cfg_.lidar_id;
                    manual_cache_.camera_id = cur_cam;
                    manual_cache_.camera_intrin = cam_intrin;
                }
                std::string result_yaml = result_subdir + "/lidar_cam_" + cur_cam + "_extrinsic.yaml";
                save_extrinsic_result(result_yaml, result, cur_cam);
                if (!lidar_scans.empty() && !camera_frames.empty()) {
                    std::string vis_path = result_subdir + "/lidar_cam_" + cur_cam + "_projection.png";
                    UNICALIB_INFO("[Viz] 即将生成投影图并保存到 {}", vis_path);
                    calibrator.visualize_projection(
                        lidar_scans[0], camera_frames[0].second,
                        *result.best(), cam_intrin, vis_path);
                    UNICALIB_INFO("[Viz] 投影图已保存（当前未将结果点云回显到 3D 窗口，仅保存 PNG）");
                }
                max_rms = std::max(max_rms, result.best_rms());
                cameras_done++;
                bool fine_converged = (result.fine.has_value() && result.fine->is_converged);
                UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 相机 {} 完成, RMS={:.4f} px  收敛={}",
                              cur_cam, result.best_rms(), fine_converged ? "是" : "否");
                if (!fine_converged && cameras_done > 0)
                    UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 精标定未收敛 (NCC 未达 0.3 或 RMS 偏高)，建议: 1) 确保粗标定提供非 identity 初值 2) 检查 LiDAR/相机 FOV 与时间同步 3) 查看上方 [精标定] 质量结论与 NCC 统计", result.best_rms());
            } else {
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 相机 {} 未得到有效结果，跳过", cur_cam);
            }
        }

#if UNICALIB_WITH_PANGOLIN
        if (lidar_cam_viz) {
            // 未启动 3D 窗口，标定结果的一帧投影已在 calibrate 内显示并阻塞至用户关闭，此处仅做收尾
            lidar_cam_viz->close_display();
        }
#endif
    }

    r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
    r.success = (cameras_done > 0);
    r.residual_rms = (cameras_done > 0) ? max_rms : -1.0;
    r.message = (cameras_done > 0) ?
        "LiDAR-Camera 精标定完成: " + std::to_string(cameras_done) + " 个相机, 最大 RMS=" + std::to_string(max_rms) + " px" :
        "LiDAR-Camera 精标定失败 — 无相机得到有效结果";
    UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 精标定完成: {} 个相机  最大 RMS={:.4f} px", cameras_done, max_rms);
    UNICALIB_INFO("[STAGE=lidar_cam_fine_auto] LiDAR-Camera 精标定 完成");
    if (cameras_done > 0) {
        bool within_threshold = (max_rms <= r.quality_threshold);
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] RMS 阈值={:.2f} px  最大 RMS={:.4f} px  {}",
                      r.quality_threshold, max_rms, within_threshold ? "在阈值内" : "超过阈值，建议手动校准");
    }
    return r;
}

// ---------------------------------------------------------------------------
// Camera-Camera 精标定：多相机按数目自动两两标定（环视+前视兼容）
// ROS2 bag 离线时若未配置 camera_topics，则自动读取 bag 内全部相机并标定
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_cam_cam() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::CAM_CAM_EXTRIN;
    r.quality_threshold = cfg_.cam_cam_rms_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    const bool use_new_format = cfg_.use_new_format;
    const bool use_ros2_bag = (!use_new_format && cfg_.use_ros2_bag && !cfg_.ros2_bag_file.empty());
    if (!use_new_format && !use_ros2_bag) {
        r.success = false;
        r.message = "Camera-Camera 需要 NEW_FORMAT 或 ROS2 bag 数据源";
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_WARN("[Fine-Auto/Cam-Cam] {}", r.message);
        return r;
    }

    // 参与标定的相机 ID：配置了则用配置；否则留空，加载 bag 后从 loader 取「全部相机」
    std::vector<std::string> camera_ids;
    if (!cfg_.cam_cam_pairs.empty()) {
        // pairs 优先：只加载配置中涉及的相机
        std::set<std::string> cam_set;
        for (const auto& pr : cfg_.cam_cam_pairs) {
            cam_set.insert(pr.first);
            cam_set.insert(pr.second);
        }
        camera_ids.assign(cam_set.begin(), cam_set.end());
    } else if (use_new_format && !cfg_.new_format_camera_index_files.empty()) {
        for (const auto& kv : cfg_.new_format_camera_index_files)
            camera_ids.push_back(kv.first);
        std::sort(camera_ids.begin(), camera_ids.end());
    } else if (!cfg_.camera_topics.empty()) {
        for (const auto& [id, _] : cfg_.camera_topics)
            camera_ids.push_back(id);
        std::sort(camera_ids.begin(), camera_ids.end());
    } else if (!cfg_.camera_id.empty()) {
        camera_ids.push_back(cfg_.camera_id);
    }

    std::map<std::string, std::vector<std::pair<double, cv::Mat>>> frames_per_cam;
    if (use_new_format) {
        UnifiedDataLoader::Config load_cfg;
        load_cfg.source_type = UnifiedDataLoader::SourceType::NEW_FORMAT;
        load_cfg.new_format_root_dir = resolve_new_format_root_dir(cfg_.new_format_root_dir);
        load_cfg.new_format_lidar_index_files = cfg_.new_format_lidar_index_files;
        load_cfg.new_format_camera_index_files = cfg_.new_format_camera_index_files;
        load_cfg.new_format_imu_index_files = cfg_.new_format_imu_index_files;
        load_cfg.new_format_timestamp_unit = cfg_.new_format_timestamp_unit;
        load_cfg.new_format_oem7_imu_rate_hz = cfg_.new_format_oem7_imu_rate_hz;
        load_cfg.new_format_oem7_time_base = cfg_.new_format_oem7_time_base;
        load_cfg.new_format_oem7_gps_utc_leap_sec = cfg_.new_format_oem7_gps_utc_leap_sec;
        load_cfg.new_format_oem7_time_offset_sec = cfg_.new_format_oem7_time_offset_sec;
        load_cfg.max_frames = cfg_.ros2_max_frames;
        load_cfg.sample_interval = cfg_.ros2_sample_interval;
        UnifiedDataLoader loader(load_cfg);
        if (!loader.load()) {
            r.success = false;
            r.message = "NEW_FORMAT 数据加载失败: " + loader.get_status_message();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        if (camera_ids.empty()) {
            camera_ids = loader.get_camera_ids();
            std::sort(camera_ids.begin(), camera_ids.end());
            UNICALIB_INFO("[Fine-Auto/Cam-Cam] NEW_FORMAT 模式，使用索引内全部相机: {} 个", camera_ids.size());
        }
        for (const auto& cid : camera_ids) {
            auto fr = loader.to_camera_frames(cid);
            if (!fr.empty())
                frames_per_cam[cid] = std::move(fr);
        }
    } else {
        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        if (!cfg_.camera_topics.empty())
            ros_cfg.camera_topics = cfg_.camera_topics;
        else if (!cfg_.camera_id.empty())
            ros_cfg.camera_topics[cfg_.camera_id] = cfg_.camera_ros2_topic;
        // camera_topics 为空时：不设置 ros_cfg.camera_topics，数据源会加载 bag 内全部 Image 话题
        ros_cfg.max_frames = cfg_.ros2_max_frames;
        UnifiedDataLoader::Config load_cfg;
        load_cfg.source_type = UnifiedDataLoader::SourceType::ROS2_BAG;
        load_cfg.ros_config = ros_cfg;
        UnifiedDataLoader loader(load_cfg);
        if (!loader.load()) {
            r.success = false;
            r.message = "ROS2 数据加载失败: " + loader.get_status_message();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        // 离线 bag 且未配置相机列表时：直接使用 loader 中已加载的全部相机
        if (camera_ids.empty()) {
            camera_ids = loader.get_camera_ids();
            std::sort(camera_ids.begin(), camera_ids.end());
            UNICALIB_INFO("[Fine-Auto/Cam-Cam] ROS2 bag 离线模式，使用 bag 内全部相机: {} 个", camera_ids.size());
        }
        for (const auto& cid : camera_ids) {
            auto fr = loader.to_camera_frames(cid);
            if (!fr.empty())
                frames_per_cam[cid] = std::move(fr);
        }
    }

    if (camera_ids.size() < 2u) {
        r.success = true;
        r.message = "Camera-Camera 标定需要至少 2 个相机，当前: " + std::to_string(camera_ids.size());
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_WARN("[Fine-Auto/Cam-Cam] {}", r.message);
        return r;
    }

    UNICALIB_INFO("[Fine-Auto/Cam-Cam] 相机数: {}，将依次标定相邻对: (0-1), (1-2), ...", camera_ids.size());

    if (frames_per_cam.size() < 2u) {
        r.success = false;
        r.message = "至少需要 2 个相机有有效帧数据，当前: " + std::to_string(frames_per_cam.size());
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_ERROR("[Fine-Auto/Cam-Cam] {}", r.message);
        return r;
    }

    // 按 camera_ids 顺序取有数据的相机，保证相邻对
    std::vector<std::string> ordered;
    for (const auto& cid : camera_ids) {
        if (frames_per_cam.count(cid))
            ordered.push_back(cid);
    }

    CamCamCalibrator::Config calib_cfg;
    calib_cfg.target.cols = cfg_.board_cols;
    calib_cfg.target.rows = cfg_.board_rows;
    calib_cfg.target.square_size_m = cfg_.square_size_m;
    calib_cfg.max_rms_px = cfg_.cam_cam_rms_threshold;
    calib_cfg.method = cfg_.prefer_targetfree ? CamCamCalibrator::Method::ESSENTIAL_MATRIX : CamCamCalibrator::Method::CHESSBOARD_STEREO;
    calib_cfg.use_initial_if_available = true;
    calib_cfg.skip_coarse_when_initial_given = cfg_.use_cam_cam_skip_coarse_when_initial;

    auto load_intrin = [this](const std::string& id) -> std::optional<CameraIntrinsics> {
        if (params_->camera_intrinsics.count(id) && params_->camera_intrinsics.at(id))
            return *params_->camera_intrinsics.at(id);
        auto it = cfg_.camera_intrinsic_files.find(id);
        if (it != cfg_.camera_intrinsic_files.end() && fs::exists(it->second)) {
            try {
                return YamlIO::load_camera_intrinsics(it->second);
            } catch (...) {}
        }
        return std::nullopt;
    };

#if UNICALIB_WITH_PANGOLIN
    CalibVisualizer::Ptr cam_cam_viz;
    if (cfg_.enable_viz) {
        try {
            cam_cam_viz = CalibVisualizer::Create();
            cam_cam_viz->start_display("UniCalib Cam-Cam", 1280, 720);
            UNICALIB_INFO("[Fine-Auto/Cam-Cam] 可视化已启用（默认）");
        } catch (const std::exception& e) {
            UNICALIB_WARN("[Fine-Auto/Cam-Cam] 可视化启动失败（继续无界面运行）: {}", e.what());
        }
    }
#endif

    int pairs_done = 0;
    double max_rms = 0.0;

    // 创建输出目录
    std::string result_subdir = cfg_.output_dir + "/camera_camera_extrinsic";
    std::error_code ec;
    fs::create_directories(result_subdir, ec);

    auto run_pair = [&](const std::string& id0, const std::string& id1) {
        if (!frames_per_cam.count(id0) || !frames_per_cam.count(id1)) {
            UNICALIB_WARN("[Fine-Auto/Cam-Cam] 跳过 {}->{}: 无帧数据", id0, id1);
            return;
        }
        auto in0 = load_intrin(id0);
        auto in1 = load_intrin(id1);
        if (!in0.has_value() || !in1.has_value()) {
            UNICALIB_WARN("[Fine-Auto/Cam-Cam] 跳过 {}->{}: 缺少内参", id0, id1);
            return;
        }
        std::optional<ExtrinsicSE3> init_extrin;
        if (cfg_.use_cam_cam_initial_from_params) {
            auto ext_ptr = params_->get_extrinsic(id0, id1);
            if (ext_ptr)
                init_extrin = *ext_ptr;
        }
        CamCamCalibrator calib(calib_cfg);
#if UNICALIB_WITH_PANGOLIN
        if (cam_cam_viz) {
            calib.set_visualizer(cam_cam_viz);
            calib.enable_realtime_viz(true);
        }
#endif
        auto result = calib.calibrate_two_stage(
            frames_per_cam.at(id0), frames_per_cam.at(id1),
            *in0, *in1, cfg_.prefer_targetfree, id0, id1, init_extrin);
        if (result.best()) {
            auto ext = params_->get_or_create_extrinsic(id0, id1);
            ext->ref_sensor_id = id0;
            ext->target_sensor_id = id1;
            ext->set_SE3(result.best()->SE3_TargetInRef());
            ext->residual_rms = result.best_rms();
            ext->is_converged = (result.best_rms() <= cfg_.cam_cam_rms_threshold);
            max_rms = std::max(max_rms, result.best_rms());
            pairs_done++;

            // 缓存首对成功数据的首帧，供手动阶段使用（手动调整仅使用第一帧匹配数据）
            if (!manual_cache_.cam0_image.has_value()) {
                const auto& f0 = frames_per_cam.at(id0);
                const auto& f1 = frames_per_cam.at(id1);
                if (!f0.empty() && !f1.empty()) {
                    manual_cache_.cam0_image = f0[0].second.clone();
                    manual_cache_.cam1_image = f1[0].second.clone();
                    manual_cache_.cam0_id = id0;
                    manual_cache_.cam1_id = id1;
                    manual_cache_.cam0_intrin = *in0;
                    manual_cache_.cam1_intrin = *in1;
                }
            }

            // 保存结果到 YAML
            std::string result_yaml = result_subdir + "/" + id0 + "_to_" + id1 + "_extrinsic.yaml";
            save_cam_cam_extrinsic_result(result_yaml, result, id0, id1);

            UNICALIB_INFO("[Fine-Auto/Cam-Cam] {} -> {} 完成, rms={:.4f} px", id0, id1, result.best_rms());
        } else {
            UNICALIB_WARN("[Fine-Auto/Cam-Cam] {} -> {} 未收敛", id0, id1);
        }
    };

    if (!cfg_.cam_cam_pairs.empty()) {
        UNICALIB_INFO("[Fine-Auto/Cam-Cam] 按配置 camera_align 标定 {} 对", cfg_.cam_cam_pairs.size());
        for (const auto& [id0, id1] : cfg_.cam_cam_pairs)
            run_pair(id0, id1);
    } else {
        for (size_t i = 0; i + 1 < ordered.size(); ++i)
            run_pair(ordered[i], ordered[i + 1]);
    }

    r.success = (pairs_done > 0);
    r.residual_rms = max_rms;
    r.message = "Camera-Camera 标定完成: " + std::to_string(pairs_done) + " 对, 最大 RMS=" + (r.success ? std::to_string(max_rms) + " px" : "N/A");
    r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
    UNICALIB_INFO("[Fine-Auto/Cam-Cam] {}", r.message);
    return r;
}

// ---------------------------------------------------------------------------
// 辅助函数: 从图像推断内参
// ---------------------------------------------------------------------------
CameraIntrinsics CalibPipeline::infer_intrinsics_from_images(
    const std::vector<std::pair<double, cv::Mat>>& frames) const {

    CameraIntrinsics intrin;
    if (frames.empty()) {
        intrin.width = 1280;
        intrin.height = 720;
        intrin.fx = intrin.width / 2.0;
        intrin.fy = intrin.width / 2.0;
        intrin.cx = intrin.width / 2.0;
        intrin.cy = intrin.height / 2.0;
        UNICALIB_WARN("[Fine-Auto] 无图像数据，使用默认 1280x720 内参");
        return intrin;
    }

    const auto& img = frames[0].second;
    intrin.width = img.cols;
    intrin.height = img.rows;
    // 假设约 90 度 FOV (f ≈ width)
    intrin.fx = intrin.width * 0.9;
    intrin.fy = intrin.width * 0.9;
    intrin.cx = intrin.width / 2.0;
    intrin.cy = intrin.height / 2.0;

    UNICALIB_INFO("[Fine-Auto] 推断内参: {}x{} fx={:.1f} fy={:.1f}",
                  intrin.width, intrin.height, intrin.fx, intrin.fy);
    return intrin;
}

// ---------------------------------------------------------------------------
// 辅助函数: 保存外参结果
// ---------------------------------------------------------------------------
void CalibPipeline::save_extrinsic_result(
    const std::string& path,
    const LiDARCameraCalibrator::TwoStageResult& result,
    const std::string& target_camera_id) const {

    ::YAML::Emitter out;
    out << ::YAML::BeginMap;
    std::string target_id = target_camera_id.empty() ? cfg_.camera_id : target_camera_id;
    out << ::YAML::Key << "calibration_type" << ::YAML::Value << "lidar_camera_extrinsic";
    out << ::YAML::Key << "reference_sensor" << ::YAML::Value << cfg_.lidar_id;
    out << ::YAML::Key << "target_sensor" << ::YAML::Value << target_id;
    out << ::YAML::Key << "timestamp" << ::YAML::Value << now_str();

    if (result.coarse.has_value()) {
        out << ::YAML::Key << "coarse_result";
        out << ::YAML::BeginMap;
        const auto& T = result.coarse->SE3_TargetInRef();
        out << ::YAML::Key << "method" << ::YAML::Value << result.coarse_method;
        out << ::YAML::Key << "rms" << ::YAML::Value << result.coarse_rms;
        out << ::YAML::Key << "translation" << ::YAML::Flow << ::YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << ::YAML::EndSeq;
        auto rpy = T.so3().log();
        out << ::YAML::Key << "rotation_rpy" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << ::YAML::EndSeq;

        // 添加 4x4 变换矩阵
        Eigen::Matrix4d mat = T.matrix();
        out << ::YAML::Key << "transformation_matrix" << ::YAML::BeginSeq;
        for (int i = 0; i < 4; ++i) {
            out << ::YAML::Flow << ::YAML::BeginSeq
                << mat(i, 0) << mat(i, 1) << mat(i, 2) << mat(i, 3) << ::YAML::EndSeq;
        }
        out << ::YAML::EndSeq;

        out << ::YAML::EndMap;
    }

    if (result.fine.has_value()) {
        out << ::YAML::Key << "fine_result";
        out << ::YAML::BeginMap;
        const auto& T = result.fine->SE3_TargetInRef();
        out << ::YAML::Key << "method" << ::YAML::Value << result.fine_method;
        out << ::YAML::Key << "rms" << ::YAML::Value << result.fine_rms;
        out << ::YAML::Key << "converged" << ::YAML::Value << result.fine->is_converged;
        out << ::YAML::Key << "translation" << ::YAML::Flow << ::YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << ::YAML::EndSeq;
        auto rpy = T.so3().log();
        out << ::YAML::Key << "rotation_rpy_rad" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << ::YAML::EndSeq;
        out << ::YAML::Key << "rotation_rpy_deg" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() * 180 / M_PI << rpy.y() * 180 / M_PI << rpy.z() * 180 / M_PI << ::YAML::EndSeq;

        // 添加 4x4 变换矩阵
        Eigen::Matrix4d mat = T.matrix();
        out << ::YAML::Key << "transformation_matrix" << ::YAML::BeginSeq;
        for (int i = 0; i < 4; ++i) {
            out << ::YAML::Flow << ::YAML::BeginSeq
                << mat(i, 0) << mat(i, 1) << mat(i, 2) << mat(i, 3) << ::YAML::EndSeq;
        }
        out << ::YAML::EndSeq;

        out << ::YAML::EndMap;
    }

    out << ::YAML::Key << "needs_manual_refine" << ::YAML::Value << result.needs_manual;
    out << ::YAML::Key << "manual_threshold_px" << ::YAML::Value << result.manual_threshold_px;

    out << ::YAML::EndMap;

    std::ofstream f(path);
    if (f.is_open()) {
        f << out.c_str();
        UNICALIB_INFO("[Fine-Auto] 结果已保存: {}", path);
    }
}

// ---------------------------------------------------------------------------
// 辅助函数: 保存 Cam-Cam 外参结果
// ---------------------------------------------------------------------------
void CalibPipeline::save_cam_cam_extrinsic_result(
    const std::string& path,
    const CamCamCalibrator::TwoStageResult& result,
    const std::string& cam0_id,
    const std::string& cam1_id) const {

    ::YAML::Emitter out;
    out << ::YAML::BeginMap;
    out << ::YAML::Key << "calibration_type" << ::YAML::Value << "camera_camera_extrinsic";
    out << ::YAML::Key << "reference_sensor" << ::YAML::Value << cam0_id;
    out << ::YAML::Key << "target_sensor" << ::YAML::Value << cam1_id;
    out << ::YAML::Key << "timestamp" << ::YAML::Value << now_str();

    if (result.coarse.has_value()) {
        out << ::YAML::Key << "coarse_result";
        out << ::YAML::BeginMap;
        const auto& T = result.coarse->SE3_TargetInRef();
        out << ::YAML::Key << "method" << ::YAML::Value << result.coarse_method;
        out << ::YAML::Key << "rms" << ::YAML::Value << result.coarse_rms;
        out << ::YAML::Key << "translation" << ::YAML::Flow << ::YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << ::YAML::EndSeq;
        auto rpy = T.so3().log();
        out << ::YAML::Key << "rotation_rpy" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << ::YAML::EndSeq;

        // 4x4 变换矩阵
        Eigen::Matrix4d mat = T.matrix();
        out << ::YAML::Key << "transformation_matrix" << ::YAML::BeginSeq;
        for (int i = 0; i < 4; ++i) {
            out << ::YAML::Flow << ::YAML::BeginSeq
                << mat(i, 0) << mat(i, 1) << mat(i, 2) << mat(i, 3) << ::YAML::EndSeq;
        }
        out << ::YAML::EndSeq;

        out << ::YAML::EndMap;
    }

    if (result.fine.has_value()) {
        out << ::YAML::Key << "fine_result";
        out << ::YAML::BeginMap;
        const auto& T = result.fine->SE3_TargetInRef();
        out << ::YAML::Key << "method" << ::YAML::Value << result.fine_method;
        out << ::YAML::Key << "rms" << ::YAML::Value << result.fine_rms;
        out << ::YAML::Key << "converged" << ::YAML::Value << (result.fine_rms <= cfg_.cam_cam_rms_threshold);
        out << ::YAML::Key << "translation" << ::YAML::Flow << ::YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << ::YAML::EndSeq;
        auto rpy = T.so3().log();
        out << ::YAML::Key << "rotation_rpy_rad" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << ::YAML::EndSeq;
        out << ::YAML::Key << "rotation_rpy_deg" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() * 180 / M_PI << rpy.y() * 180 / M_PI << rpy.z() * 180 / M_PI << ::YAML::EndSeq;

        // 4x4 变换矩阵
        Eigen::Matrix4d mat = T.matrix();
        out << ::YAML::Key << "transformation_matrix" << ::YAML::BeginSeq;
        for (int i = 0; i < 4; ++i) {
            out << ::YAML::Flow << ::YAML::BeginSeq
                << mat(i, 0) << mat(i, 1) << mat(i, 2) << mat(i, 3) << ::YAML::EndSeq;
        }
        out << ::YAML::EndSeq;

        out << ::YAML::EndMap;
    }

    out << ::YAML::Key << "needs_manual_refine" << ::YAML::Value << result.needs_manual;
    out << ::YAML::Key << "manual_threshold_px" << ::YAML::Value << result.manual_threshold_px;

    out << ::YAML::EndMap;

    std::ofstream f(path);
    if (f.is_open()) {
        f << out.c_str();
        UNICALIB_INFO("[Fine-Auto/Cam-Cam] 结果已保存: {}", path);
    }
}

// ---------------------------------------------------------------------------
// IMU 内参精标定实现
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_imu_intrinsic() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::IMU_INTRINSIC;
    r.quality_threshold = cfg_.imu_intrin_rms_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    UNICALIB_INFO("[Fine-Auto/IMU-Intrin] 开始执行IMU内参标定...");

    // ─── 1. 数据加载 ─────────────────────────────────────────────────
    ns_unicalib::IMURawData imu_data;

    if (cfg_.use_new_format) {
        UNICALIB_INFO("  数据源类型: NEW_FORMAT");
        const std::string nf_root_resolved = resolve_new_format_root_dir(cfg_.new_format_root_dir);
        UNICALIB_INFO("  NEW_FORMAT 根目录: {} (解析后: {})",
                      cfg_.new_format_root_dir.empty() ? "(未设置)" : cfg_.new_format_root_dir,
                      nf_root_resolved.empty() ? "(未设置)" : nf_root_resolved);

        if (nf_root_resolved.empty() || !fs::exists(nf_root_resolved)) {
            r.success = false;
            r.message = "NEW_FORMAT 根目录未配置或不存在: " +
                        (nf_root_resolved.empty() ? cfg_.new_format_root_dir : nf_root_resolved);
            return r;
        }

        UnifiedDataLoader::Config unified_cfg;
        unified_cfg.source_type = UnifiedDataLoader::SourceType::NEW_FORMAT;
        unified_cfg.new_format_root_dir = nf_root_resolved;
        unified_cfg.new_format_lidar_index_files = cfg_.new_format_lidar_index_files;
        unified_cfg.new_format_camera_index_files = cfg_.new_format_camera_index_files;
        unified_cfg.new_format_imu_index_files = cfg_.new_format_imu_index_files;
        unified_cfg.new_format_timestamp_unit = cfg_.new_format_timestamp_unit;
        unified_cfg.new_format_oem7_imu_rate_hz = cfg_.new_format_oem7_imu_rate_hz;
        unified_cfg.new_format_oem7_time_base = cfg_.new_format_oem7_time_base;
        unified_cfg.new_format_oem7_gps_utc_leap_sec = cfg_.new_format_oem7_gps_utc_leap_sec;
        unified_cfg.new_format_oem7_time_offset_sec = cfg_.new_format_oem7_time_offset_sec;
        unified_cfg.max_frames = 0;  // IMU 内参希望尽量使用全量静止数据

        UnifiedDataLoader loader(unified_cfg);
        if (!loader.load()) {
            r.success = false;
            r.message = "NEW_FORMAT 数据加载失败: " + loader.get_status_message();
            return r;
        }

        imu_data = loader.to_imu_raw_data(cfg_.imu_sensor_id);
        UNICALIB_INFO("  加载IMU数据: {} 帧 (sensor_id={})", imu_data.size(), cfg_.imu_sensor_id);

    } else if (cfg_.use_ros2_bag && !cfg_.ros2_bag_file.empty()) {
        UNICALIB_INFO("  数据源类型: ROS2 Bag 文件");
        UNICALIB_INFO("  ROS2 Bag 文件: {}", cfg_.ros2_bag_file);

        if (!fs::exists(cfg_.ros2_bag_file)) {
            r.success = false;
            r.message = "ROS2 Bag 路径不存在: " + cfg_.ros2_bag_file;
            return r;
        }

        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.realtime_mode = false;
        ros_cfg.max_frames = 0;  // 不限制，IMU 内参需要尽量多静置数据
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        if (!cfg_.imu_topics.empty()) {
            ros_cfg.imu_topics = cfg_.imu_topics;
            ros_cfg.imu_ros2_topic = cfg_.imu_topics.count(cfg_.imu_sensor_id) ?
                cfg_.imu_topics.at(cfg_.imu_sensor_id) : cfg_.imu_topics.begin()->second;
        } else if (!cfg_.imu_ros2_topic.empty()) {
            ros_cfg.imu_topics[cfg_.imu_sensor_id] = cfg_.imu_ros2_topic;
            ros_cfg.imu_ros2_topic = cfg_.imu_ros2_topic;
        }

        UnifiedDataLoader::Config unified_cfg;
        unified_cfg.source_type = UnifiedDataLoader::SourceType::ROS2_BAG;
        unified_cfg.ros_config = ros_cfg;
        unified_cfg.max_frames = 0;

        UnifiedDataLoader loader(unified_cfg);
        if (!loader.load()) {
            r.success = false;
            r.message = "ROS2 数据加载失败: " + loader.get_status_message();
            return r;
        }

        imu_data = loader.to_imu_raw_data(cfg_.imu_sensor_id);
        UNICALIB_INFO("  加载IMU数据: {} 帧 (sensor_id={})", imu_data.size(), cfg_.imu_sensor_id);

    } else if (!cfg_.imu_data_file.empty()) {
        UNICALIB_INFO("  数据源类型: CSV文件");
        UNICALIB_INFO("  IMU数据文件: {}", cfg_.imu_data_file);
        
        // TODO: 实现CSV加载功能
        UNICALIB_ERROR("CSV数据加载功能暂未实现");
        r.success = false;
        r.message = "CSV数据加载未实现";
        return r;
    } else {
        UNICALIB_ERROR("未指定数据源 (ros2_bag_file 或 imu_data_file)");
        r.success = false;
        r.message = "数据源未配置";
        return r;
    }

    if (imu_data.empty()) {
        UNICALIB_ERROR("IMU数据为空，请检查ROS2 bag文件是否包含IMU数据");
        r.success = false;
        r.message = "IMU数据加载失败或bag文件中无IMU数据";
        return r;
    }

    // ─── 2. Allan方差分析 ─────────────────────────────────────────────
    UNICALIB_LOG_STEP("IMU-Intrin", "Step 1: Allan 方差分析...");
    
    ns_unicalib::IMUIntrinsicCalibrator calibrator(cfg_.imu_intrinsic_calib_cfg);
    
    if (progress_cb_) {
        calibrator.set_progress_callback([&](const std::string& stage, double progress) {
            progress_cb_(CalibStage::FINE_AUTO, "Allan方差分析: " + stage, 0.1 + 0.6 * progress);
        });
    }
    
    auto intrinsics = calibrator.calibrate(imu_data);

    // ─── 3. 保存结果 ─────────────────────────────────────────────────
    UNICALIB_LOG_STEP("IMU-Intrin", "Step 2: 保存标定结果...");

    std::string output_yaml;
    if (!cfg_.results_imu_intrinsic.empty()) {
        std::string subdir = cfg_.output_dir + "/" + cfg_.results_imu_intrinsic;
        std::error_code ec;
        fs::create_directories(subdir, ec);
        output_yaml = subdir + "/imu_intrinsic_" + cfg_.imu_sensor_id + ".yaml";
    } else {
        output_yaml = cfg_.output_dir + "/imu_intrinsic.yaml";
    }
    save_imu_intrinsic_yaml(intrinsics, output_yaml);

    // 保存 Allan 偏差图（analyze_allan 与 calibrate 内部逻辑一致，仅用于绘图）
    ns_unicalib::AllanResult allan_result = calibrator.analyze_allan(imu_data);
    if (!allan_result.gyro[0].taus.empty()) {
        std::string plot_path = cfg_.output_dir + "/imu_allan_variance.png";
        calibrator.save_allan_plot(allan_result, plot_path);
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    r.success = true;
    r.residual_rms = intrinsics.allan_fit_rms;
    std::ostringstream msg_oss;
    msg_oss << "IMU内参标定完成: gyro_noise=" << intrinsics.noise_gyro
            << " rad/s/√Hz, gyro_bias=" << intrinsics.bias_instab_gyro
            << " rad/s, accel_noise=" << intrinsics.noise_acce
            << " m/s²/√Hz, accel_bias=" << intrinsics.bias_instab_acce << " m/s²";
    r.message = msg_oss.str();
    
    UNICALIB_INFO("[Fine-Auto/IMU-Intrin] 标定完成: RMS={:.4f} | 耗时: {:.1f} ms", 
                  r.residual_rms, r.elapsed_ms);
    return r;
}

// ---------------------------------------------------------------------------
// IMU-LiDAR 外参精标定（支持 imu_lidar.pairs 多对）
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_imu_lidar() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::IMU_LIDAR_EXTRIN;
    r.quality_threshold = cfg_.imu_lidar_rot_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    // 标定对：优先 cfg_.imu_lidar_pairs，否则取第一个 IMU + 第一个 LiDAR
    std::vector<std::pair<std::string, std::string>> pairs = cfg_.imu_lidar_pairs;
    if (pairs.empty()) {
        std::string imu_id = cfg_.reference_imu.empty() ? cfg_.imu_sensor_id : cfg_.reference_imu;
        if (cfg_.imu_topics.count(imu_id)) { /* use it */ } else if (!cfg_.imu_topics.empty())
            imu_id = cfg_.imu_topics.begin()->first;
        std::string lidar_id = cfg_.lidar_id;
        if (cfg_.lidar_topics.count(lidar_id)) { /* use it */ } else if (!cfg_.lidar_topics.empty())
            lidar_id = cfg_.lidar_topics.begin()->first;
        if (!imu_id.empty() && !lidar_id.empty())
            pairs.emplace_back(imu_id, lidar_id);
    }
    if (pairs.empty()) {
        r.success = false;
        r.message = "未配置 IMU-LiDAR 标定对 (imu_lidar.pairs 或 sensors)";
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_ERROR("[Fine-Auto/IMU-LiDAR] {}", r.message);
        return r;
    }

    UnifiedDataLoader::Config load_cfg;
    if (cfg_.use_new_format) {
        load_cfg.source_type = UnifiedDataLoader::SourceType::NEW_FORMAT;
        load_cfg.new_format_root_dir = resolve_new_format_root_dir(cfg_.new_format_root_dir);
        load_cfg.new_format_lidar_index_files = cfg_.new_format_lidar_index_files;
        load_cfg.new_format_camera_index_files = cfg_.new_format_camera_index_files;
        load_cfg.new_format_imu_index_files = cfg_.new_format_imu_index_files;
        load_cfg.new_format_timestamp_unit = cfg_.new_format_timestamp_unit;
        load_cfg.new_format_oem7_imu_rate_hz = cfg_.new_format_oem7_imu_rate_hz;
        load_cfg.new_format_oem7_time_base = cfg_.new_format_oem7_time_base;
        load_cfg.new_format_oem7_gps_utc_leap_sec = cfg_.new_format_oem7_gps_utc_leap_sec;
        load_cfg.new_format_oem7_time_offset_sec = cfg_.new_format_oem7_time_offset_sec;
        load_cfg.max_frames = cfg_.ros2_max_frames;
        load_cfg.sample_interval = cfg_.ros2_sample_interval;
        if (load_cfg.new_format_root_dir.empty() || !fs::exists(load_cfg.new_format_root_dir)) {
            r.success = false;
            r.message = "IMU-LiDAR NEW_FORMAT 根目录无效或不存在: " + load_cfg.new_format_root_dir;
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
    } else {
        if (!cfg_.use_ros2_bag || cfg_.ros2_bag_file.empty()) {
            r.success = false;
            r.message = "IMU-LiDAR 精标定当前需 NEW_FORMAT 或 ROS2 bag 数据源";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/IMU-LiDAR] {}", r.message);
            return r;
        }
        if (!fs::exists(cfg_.ros2_bag_file)) {
            r.success = false;
            r.message = "ROS2 Bag 路径不存在: " + cfg_.ros2_bag_file;
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.realtime_mode = false;
        ros_cfg.max_frames = cfg_.ros2_max_frames;
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        ros_cfg.imu_topics = cfg_.imu_topics;
        ros_cfg.lidar_topics = cfg_.lidar_topics;
        if (ros_cfg.imu_topics.empty() && !cfg_.imu_ros2_topic.empty())
            ros_cfg.imu_topics["imu_0"] = cfg_.imu_ros2_topic;
        if (ros_cfg.lidar_topics.empty() && !cfg_.lidar_ros2_topic.empty())
            ros_cfg.lidar_topics[cfg_.lidar_id] = cfg_.lidar_ros2_topic;
        load_cfg.source_type = UnifiedDataLoader::SourceType::ROS2_BAG;
        load_cfg.ros_config = ros_cfg;
        load_cfg.max_frames = ros_cfg.max_frames;
    }

    UnifiedDataLoader loader(load_cfg);
    if (!loader.load()) {
        r.success = false;
        r.message = (cfg_.use_new_format ? "NEW_FORMAT 数据加载失败: " : "ROS2 数据加载失败: ") + loader.get_status_message();
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        return r;
    }

    IMULiDARCalibrator::Config calib_cfg;
    calib_cfg.ndt_resolution = 1.0;
    calib_cfg.ndt_max_iter = 30;
    calib_cfg.spline_dt_s = 0.1;
    calib_cfg.spline_order = 4;
    calib_cfg.optimize_time_offset = true;
    calib_cfg.time_offset_init_s = 0.0;
    calib_cfg.time_offset_max_s = 0.2;
    calib_cfg.min_motion_rot_deg = 3.0;
    calib_cfg.rot_pair_dt_target_s = 0.2;      // 短时窗降低陀螺积分漂移
    calib_cfg.min_motion_rot_deg_short = 0.5;
    calib_cfg.rot_pair_short_step_s = 0.1;
    calib_cfg.handeye_fix_180_ambiguity = true;
    calib_cfg.handeye_180_decision = cfg_.imu_lidar_handeye_180_decision.empty() ? "prefer_identity" : cfg_.imu_lidar_handeye_180_decision;
    calib_cfg.handeye_prefer_identity_when_ambiguous = cfg_.imu_lidar_handeye_prefer_identity_when_ambiguous;
    calib_cfg.handeye_180_residual_margin_deg = cfg_.imu_lidar_handeye_180_residual_margin_deg;
    calib_cfg.use_planar_prior = true;           // 车辆等激励不足时 roll/pitch 与 trans_z 用先验
    calib_cfg.min_roll_motion_deg = 5.0;
    calib_cfg.min_pitch_motion_deg = 5.0;
    calib_cfg.min_yaw_motion_deg = 5.0;
    calib_cfg.max_z_trans_ratio = 0.2;
    calib_cfg.enable_planar_warning = true;
    calib_cfg.verbose = (cfg_.log_level == "debug" || cfg_.log_level == "trace");

    std::string result_subdir = cfg_.output_dir + "/imu_lidar_extrinsic";
    fs::create_directories(result_subdir);

    int pairs_done = 0;
    double max_rot_err = 0.0;
    for (const auto& [imu_id, lidar_id] : pairs) {
        IMURawData raw_imu = loader.to_imu_raw_data(imu_id);
        std::vector<LiDARScan> lidar_scans = loader.to_lidar_scans(lidar_id);
        if (raw_imu.empty() || lidar_scans.empty()) {
            UNICALIB_WARN("[Fine-Auto/IMU-LiDAR] 跳过 {}->{}: IMU {} 帧, LiDAR {} 帧",
                          imu_id, lidar_id, raw_imu.size(), lidar_scans.size());
            continue;
        }
        std::vector<IMUFrame> imu_frames;
        imu_frames.reserve(raw_imu.size());
        for (const auto& fr : raw_imu) {
            IMUFrame f;
            f.timestamp = fr.timestamp;
            f.gyro = fr.gyro;
            f.accel = fr.accel;
            imu_frames.push_back(f);
        }
        const IMUIntrinsics* imu_intrin = nullptr;
        std::optional<IMUIntrinsics> loaded_imu_intrin;
        if (params_ && params_->imu_intrinsics.count(imu_id) && params_->imu_intrinsics.at(imu_id)) {
            imu_intrin = params_->imu_intrinsics.at(imu_id).get();
            UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {} 使用已加载的 IMU 内参 (params)", imu_id);
        } else {
            auto it = cfg_.imu_intrinsic_files.find(imu_id);
            if (it != cfg_.imu_intrinsic_files.end()) {
                if (fs::exists(it->second)) {
                    try {
                        loaded_imu_intrin = YamlIO::load_imu_intrinsics(it->second);
                        imu_intrin = &(*loaded_imu_intrin);
                        UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {} 从内参标定结果文件加载: {}", imu_id, it->second);
                    } catch (const std::exception& e) {
                        UNICALIB_WARN("[Fine-Auto/IMU-LiDAR] {} 加载 IMU 内参失败 {}: {}", imu_id, it->second, e.what());
                    }
                } else {
                    UNICALIB_WARN("[Fine-Auto/IMU-LiDAR] {} 内参文件不存在，将不使用陀螺零偏: {}", imu_id, it->second);
                }
            } else {
                UNICALIB_WARN("[Fine-Auto/IMU-LiDAR] {} 未配置 IMU 内参路径 (results.imu_intrinsic / imu_intrinsic_files)，将不使用陀螺零偏", imu_id);
            }
        }

        // 初值：优先内联 4x4，否则从文件加载；有则作为 coarse_init 在此基础上精化
        std::optional<Sophus::SE3d> coarse_init;
        {
            const std::string key = imu_id + "__" + lidar_id;
            const std::string key_alt = "T_" + key;
            auto it_inline = cfg_.imu_lidar_initial_extrinsic_inline.find(key);
            if (it_inline == cfg_.imu_lidar_initial_extrinsic_inline.end())
                it_inline = cfg_.imu_lidar_initial_extrinsic_inline.find(key_alt);
            if (it_inline != cfg_.imu_lidar_initial_extrinsic_inline.end() && it_inline->second.size() >= 16u) {
                const auto& d = it_inline->second;
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) T(i, j) = d[i * 4 + j];
                Eigen::Matrix3d R = project_to_rotation(T.block<3,3>(0,0));
                coarse_init = Sophus::SE3d(R, T.block<3,1>(0,3));
                UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {}->{} 使用配置内联初值 (在初值基础上精化)", imu_id, lidar_id);
            } else {
                auto it = cfg_.imu_lidar_initial_extrinsic_files.find(key);
                if (it == cfg_.imu_lidar_initial_extrinsic_files.end()) it = cfg_.imu_lidar_initial_extrinsic_files.find(key_alt);
                if (it != cfg_.imu_lidar_initial_extrinsic_files.end() && !it->second.empty()) {
                    fs::path p(it->second);
                    if (!p.is_absolute()) p = fs::current_path() / p;
                    auto se3 = load_se3_from_imu_lidar_yaml(p.string());
                    if (se3) {
                        coarse_init = *se3;
                        UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {}->{} 使用初值文件: {} (在初值基础上精化)", imu_id, lidar_id, p.string());
                    } else {
                        UNICALIB_WARN("[Fine-Auto/IMU-LiDAR] {}->{} 初值文件加载失败，改用手眼初始化: {}", imu_id, lidar_id, p.string());
                    }
                }
            }
        }
        IMULiDARCalibrator calibrator(calib_cfg);
        auto result = calibrator.calibrate_two_stage(imu_frames, lidar_scans, imu_id, lidar_id, imu_intrin, coarse_init);
        const ExtrinsicSE3* best = result.best();
        if (!best) {
            UNICALIB_WARN("[Fine-Auto/IMU-LiDAR] {}->{} 未收敛", imu_id, lidar_id);
            continue;
        }
        if (params_) {
            auto ext = params_->get_or_create_extrinsic(imu_id, lidar_id);
            if (ext) *ext = *best;
        }
        // 日志输出最终 4x4 外参矩阵，便于分析标定质量
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,3>(0,0) = best->SO3_TargetInRef.matrix();
            T.block<3,1>(0,3) = best->POS_TargetInRef;
            UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {}->{} 最终 4x4 T_Imu_Lidar (p_imu = T * p_lidar):", imu_id, lidar_id);
            UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(0,0), T(0,1), T(0,2), T(0,3));
            UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(1,0), T(1,1), T(1,2), T(1,3));
            UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(2,0), T(2,1), T(2,2), T(2,3));
            UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(3,0), T(3,1), T(3,2), T(3,3));
        }
        std::string out_path = result_subdir + "/imu_lidar_" + imu_id + "_" + lidar_id + ".yaml";
        const std::string out_path_abs = fs::absolute(out_path).string();
        std::ofstream of(out_path);
        if (of.is_open()) {
            UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {}->{} 结果已写入: {}", imu_id, lidar_id, out_path_abs);
            of << "calibration_type: imu_lidar_extrinsic\n";
            of << "ref_sensor: " << imu_id << "\ntarget_sensor: " << lidar_id << "\n";
            // T_Imu_Lidar: 4x4 刚体变换 (与 imu_intrinsic 中 T_Body_Imu 格式一致)
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,3>(0,0) = best->SO3_TargetInRef.matrix();
            T.block<3,1>(0,3) = best->POS_TargetInRef;
            of << "# T_Imu_Lidar: IMU 系到 LiDAR 系的 4x4 刚体变换矩阵 (Imu -> Lidar)\n";
            of << "# 含义: 将 LiDAR 坐标系下的点 p_lidar 变换到 IMU 系: p_imu = T_Imu_Lidar * p_lidar\n";
            of << "T_Imu_Lidar:\n";
            of << "  rows: 4\n  cols: 4\n  dt: d\n";
            of << "  data: [ ";
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (i * 4 + j > 0) of << ", ";
                    of << T(i, j);
                }
                if (i < 3) of << ",\n           ";
            }
            of << " ]\n";
            of << "time_offset_s: " << best->time_offset_s << "\n";
            // 标定精度指标（用于评价与追溯）
            if (best->handeye_rot_residual_deg >= 0.0 || best->trans_rms_m_s >= 0.0 || best->bspline_final_cost >= 0.0) {
                of << "# 标定精度 (accuracy)\n";
                of << "accuracy:\n";
                if (best->handeye_rot_residual_deg >= 0.0)
                    of << "  handeye_rot_residual_deg: " << best->handeye_rot_residual_deg << "  # 手眼旋转平均残差 [deg]\n";
                if (best->trans_rms_m_s >= 0.0)
                    of << "  trans_rms_m_s: " << best->trans_rms_m_s << "  # 平移估计 RMS [m/s]\n";
                if (best->bspline_final_cost >= 0.0)
                    of << "  bspline_final_cost: " << best->bspline_final_cost << "  # B样条优化最终代价\n";
            }
        }
        pairs_done++;
        UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {}->{} 完成", imu_id, lidar_id);
    }

    r.success = (pairs_done > 0);
    r.residual_rms = max_rot_err;
    r.message = "IMU-LiDAR 标定完成: " + std::to_string(pairs_done) + " 对";
    r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
    UNICALIB_INFO("[Fine-Auto/IMU-LiDAR] {}", r.message);
    return r;
}

// ---------------------------------------------------------------------------
// 保存 IMU 内参结果到 YAML
// ---------------------------------------------------------------------------
void CalibPipeline::save_imu_intrinsic_yaml(const ns_unicalib::IMUIntrinsics& intrinsics,
                                           const std::string& path) const {
    ::YAML::Emitter out;
    out << ::YAML::BeginMap;
    out << ::YAML::Key << "imu_id" << ::YAML::Value << cfg_.imu_sensor_id;

    // 陀螺仪参数 (noise_density: rad/s/√Hz, bias_instability: rad/s)
    out << ::YAML::Key << "gyroscope" << ::YAML::BeginMap;
    out << ::YAML::Key << "noise_density" << ::YAML::Value << intrinsics.noise_gyro;
    out << ::YAML::Key << "bias_instability" << ::YAML::Value << intrinsics.bias_instab_gyro;
    out << ::YAML::Key << "random_walk" << ::YAML::Value << 0.0;
    out << ::YAML::EndMap;

    // 加速度计参数 (noise_density: m/s²/√Hz, bias_instability: m/s²)
    out << ::YAML::Key << "accelerometer" << ::YAML::BeginMap;
    out << ::YAML::Key << "noise_density" << ::YAML::Value << intrinsics.noise_acce;
    out << ::YAML::Key << "bias_instability" << ::YAML::Value << intrinsics.bias_instab_acce;
    out << ::YAML::Key << "random_walk" << ::YAML::Value << 0.0;
    out << ::YAML::EndMap;

    // 元数据
    out << ::YAML::Key << "metadata" << ::YAML::BeginMap;
    out << ::YAML::Key << "method" << ::YAML::Value << "allan_variance";
    out << ::YAML::Key << "num_samples" << ::YAML::Value << intrinsics.num_samples_used;
    out << ::YAML::Key << "allan_fit_rms" << ::YAML::Value << intrinsics.allan_fit_rms;
    out << ::YAML::EndMap;

    out << ::YAML::EndMap;

    std::ofstream f(path);
    if (f.is_open()) {
        f << out.c_str();
        UNICALIB_INFO("[Fine-Auto] IMU内参已保存: {}", path);
    } else {
        UNICALIB_ERROR("[Fine-Auto] 无法保存IMU内参到: {}", path);
    }
}

// ---------------------------------------------------------------------------
// 手动校准阶段入口：调用 ManualCalibSession 进行 6-DOF 交互调整
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_manual_stage(CalibTaskType task) {
    StageResult r;
    r.stage = CalibStage::MANUAL_REFINE;
    r.task  = task;
    auto t_start = std::chrono::high_resolution_clock::now();

    UNICALIB_INFO("[Manual-Refine] 任务: {}", task_str(task));
    UNICALIB_INFO("[Manual-Refine] 操作说明: Q/A Roll  W/S Pitch  E/D Yaw  R/F Tx  T/G Ty  Y/H Tz  U Undo  Enter 接受  Esc 取消");

    // 使用 --manual 时进入此阶段，始终打开 6-DOF 可视化调整窗口
    ManualCalibSession::SessionConfig sess_cfg;
    sess_cfg.save_dir = cfg_.output_dir + "/manual_sessions";
    sess_cfg.enable_interactive_gui = true;  // --manual 即打开可视化界面供手动调整
    sess_cfg.auto_save = true;
    sess_cfg.use_pangolin_manual_panel = cfg_.use_pangolin_manual_panel;
    ManualCalibSession session(sess_cfg);

    if (task == CalibTaskType::LIDAR_CAM_EXTRIN) {
        if (!manual_cache_.lidar_scan.has_value() || !manual_cache_.camera_image.has_value() || !manual_cache_.camera_intrin.has_value() || !params_) {
            r.success = false;
            r.message = "无 LiDAR-Camera 精标定缓存数据，无法启用手动调整（请先完成精标定）";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            UNICALIB_WARN("[Manual-Refine] {}", r.message);
            return r;
        }
        UNICALIB_INFO("[Manual-Refine] 已打开 6-DOF 可视化调整窗口，请在本窗口内按键微调后 Enter 接受或 Esc 取消");
        UNICALIB_INFO("[Manual-Refine] LiDAR-Camera 手动调整仅使用首帧匹配数据（第 0 帧点云 + 第 0 帧图像）");
        auto ext_ptr = params_->get_extrinsic(manual_cache_.lidar_id, manual_cache_.camera_id);
        ExtrinsicSE3 auto_ext = ext_ptr ? *ext_ptr : ExtrinsicSE3();
        double auto_rms = ext_ptr && ext_ptr->residual_rms >= 0 ? ext_ptr->residual_rms : 0.0;
        ExtrinsicSE3 result = session.run_lidar_cam(
            *manual_cache_.lidar_scan, *manual_cache_.camera_image,
            *manual_cache_.camera_intrin, auto_ext, auto_rms);
        auto out = params_->get_or_create_extrinsic(manual_cache_.lidar_id, manual_cache_.camera_id);
        if (out) *out = result;
        r.residual_rms = result.residual_rms;
        r.success = true;
        r.message = "LiDAR-Camera 手动调整完成";
    } else if (task == CalibTaskType::CAM_CAM_EXTRIN) {
        if (!manual_cache_.cam0_image.has_value() || !manual_cache_.cam1_image.has_value() || !manual_cache_.cam0_intrin.has_value() || !manual_cache_.cam1_intrin.has_value() || !params_) {
            r.success = false;
            r.message = "无 Camera-Camera 精标定缓存数据，无法启用手动调整";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            UNICALIB_WARN("[Manual-Refine] {}", r.message);
            return r;
        }
        UNICALIB_INFO("[Manual-Refine] 已打开 6-DOF 可视化调整窗口，请在本窗口内按键微调后 Enter 接受或 Esc 取消");
        UNICALIB_INFO("[Manual-Refine] Camera-Camera 手动调整仅使用首帧匹配数据（双相机各第 0 帧）");
        auto ext_ptr = params_->get_extrinsic(manual_cache_.cam0_id, manual_cache_.cam1_id);
        ExtrinsicSE3 auto_ext = ext_ptr ? *ext_ptr : ExtrinsicSE3();
        double auto_rms = ext_ptr && ext_ptr->residual_rms >= 0 ? ext_ptr->residual_rms : 0.0;
        ExtrinsicSE3 result = session.run_cam_cam(
            *manual_cache_.cam0_image, *manual_cache_.cam1_image,
            *manual_cache_.cam0_intrin, *manual_cache_.cam1_intrin,
            auto_ext, auto_rms);
        auto out = params_->get_or_create_extrinsic(manual_cache_.cam0_id, manual_cache_.cam1_id);
        if (out) *out = result;
        r.residual_rms = result.residual_rms;
        r.success = true;
        r.message = "Camera-Camera 手动调整完成";
    } else if (task == CalibTaskType::LIDAR_LIDAR_EXTRIN) {
        if (!params_ || cfg_.lidar_lidar_pairs.empty()) {
            r.success = false;
            r.message = "无 LiDAR-LiDAR 标定对或参数管理器，无法启用手动调整";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        const auto& [ref_id, target_id] = cfg_.lidar_lidar_pairs.front();
        auto ext_ptr = params_->get_extrinsic(ref_id, target_id);
        ExtrinsicSE3 auto_ext = ext_ptr ? *ext_ptr : ExtrinsicSE3();
        double auto_rms = ext_ptr && ext_ptr->residual_rms >= 0 ? ext_ptr->residual_rms : (ext_ptr && ext_ptr->is_converged ? 0.5 : 0.0);
        const LiDARScan* ref_first = manual_cache_.lidar_lidar_ref_scan.has_value() ? &*manual_cache_.lidar_lidar_ref_scan : nullptr;
        const LiDARScan* target_first = manual_cache_.lidar_lidar_target_scan.has_value() ? &*manual_cache_.lidar_lidar_target_scan : nullptr;
        if (ref_first && target_first) {
            UNICALIB_INFO("[Manual-Stage] LiDAR-LiDAR 使用缓存的一对点云（{} + {} ，|Δt|={:.6f}s）",
                          ref_id, target_id,
                          std::fabs(ref_first->timestamp - target_first->timestamp));
        }
        ExtrinsicSE3 result = session.run_lidar_lidar(auto_ext, auto_rms, ref_first, target_first);
        auto out = params_->get_or_create_extrinsic(ref_id, target_id);
        if (out) *out = result;
        r.residual_rms = result.residual_rms >= 0 ? result.residual_rms : 0.0;
        r.success = true;
        r.message = "LiDAR-LiDAR 手动调整完成";
    } else if (task == CalibTaskType::IMU_LIDAR_EXTRIN) {
        if (!params_ || cfg_.imu_lidar_pairs.empty()) {
            r.success = false;
            r.message = "无 IMU-LiDAR 标定对，无法启用手动调整";
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        const auto& [imu_id, lidar_id] = cfg_.imu_lidar_pairs.front();
        auto ext_ptr = params_->get_extrinsic(imu_id, lidar_id);
        ExtrinsicSE3 auto_ext = ext_ptr ? *ext_ptr : ExtrinsicSE3();
        double auto_rms = ext_ptr && ext_ptr->handeye_rot_residual_deg >= 0 ? ext_ptr->handeye_rot_residual_deg : 0.0;
        std::vector<LiDARScan> empty_scans;
        std::vector<IMUFrame> empty_imu;
        ExtrinsicSE3 result = session.run_imu_lidar(empty_scans, empty_imu, auto_ext, auto_rms);
        auto out = params_->get_or_create_extrinsic(imu_id, lidar_id);
        if (out) *out = result;
        r.residual_rms = auto_rms;
        r.success = true;
        r.message = "IMU-LiDAR 手动调整完成";
    } else {
        r.success = false;
        r.message = "手动校准不支持任务类型: " + task_str(task);
    }

    r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
    return r;
}

}  // namespace ns_unicalib
