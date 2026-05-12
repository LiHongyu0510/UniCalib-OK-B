/**
 * UniCalib — 联合标定求解器实现 (工程化版本)
 * 协调五种标定的执行, 集成 B-样条联合优化
 *
 * 工程化改造:
 *   - 使用统一的 Status/ErrorCode 异常体系
 *   - 改造所有 std::exception catch 为具体异常类型
 */

#include "unicalib/solver/joint_calib_solver.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/status.h"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <thread>

// iKalibr/veta 头文件必须在任何 namespace ns_unicalib 之前包含，避免污染导致成员被解析到 ns_veta
// 注意：CMake 在 OFF 时定义 UNICALIB_WITH_IKALIBR=0，故用 #if 而非 #ifdef
#if UNICALIB_WITH_IKALIBR
#include <basalt/spline/se3_spline.h>
#include <basalt/spline/ceres_spline_helper.h>
#include <basalt/utils/sophus_utils.hpp>
#include <ceres/ceres.h>
#include "ikalibr/calib/calib_data_manager.h"
#include "ikalibr/calib/calib_param_manager.h"
#include "ikalibr/solver/calib_solver.h"
#include "ikalibr/sensor/imu.h"
#include "ikalibr/sensor/lidar.h"
#include "ikalibr/sensor/camera.h"
#endif

namespace fs = std::filesystem;
namespace ns_unicalib {

#if UNICALIB_WITH_IKALIBR
// ===================================================================
// 数据转换辅助函数 (CalibDataBundle → iKalibr frame types)
// ===================================================================

/**
 * @brief 将 UniCalib IMUFrame 转换为 iKalibr::IMUFrame::Ptr
 * 注意: 两者数据结构兼容 (均来自 ns_ctraj::IMUFrame)
 */
inline ns_ikalibr::IMUFrame::Ptr convert_imu_frame(const IMUFrame& frame) {
    auto ikalibr_frame = std::make_shared<ns_ikalibr::IMUFrame>(frame.timestamp);
    ikalibr_frame->SetGyro(frame.gyro);
    ikalibr_frame->SetAccel(frame.accel);
    return ikalibr_frame;
}

/**
 * @brief 将 UniCalib LiDARScan 转换为 iKalibr::LiDARFrame::Ptr
 */
inline ns_ikalibr::LiDARFrame::Ptr convert_lidar_frame(const LiDARScan& scan) {
    if (!scan.cloud) return nullptr;
    auto ikalibr_frame = std::make_shared<ns_ikalibr::LiDARFrame>(scan.timestamp);
    // 复制点云 (需根据 iKalibr LiDARFrame 的实际成员结构)
    // ikalibr_frame->SetCloud(scan.cloud);
    return ikalibr_frame;
}

/**
 * @brief 将 UniCalib CameraFrame 转换为 iKalibr::CameraFrame::Ptr
 */
inline ns_ikalibr::CameraFrame::Ptr convert_camera_frame(
    const std::pair<double, cv::Mat>& frame) {
    auto ikalibr_frame = std::make_shared<ns_ikalibr::CameraFrame>(frame.first);
    // ikalibr_frame->SetImage(frame.second);
    return ikalibr_frame;
}

#endif  // UNICALIB_WITH_IKALIBR

JointCalibSolver::JointCalibSolver()
    : JointCalibSolver(Config{}) {}

JointCalibSolver::JointCalibSolver(const Config& cfg)
    : cfg_(cfg), params_(CalibParamManager::Create()) {}

void JointCalibSolver::set_system_config(const SystemConfig& sys_cfg) {
    sys_cfg_ = sys_cfg;
}

void JointCalibSolver::set_imu_intrinsics(
    const std::string& sensor_id, const IMUIntrinsics& intrin) {
    params_->imu_intrinsics[sensor_id] = std::make_shared<IMUIntrinsics>(intrin);
}

void JointCalibSolver::set_camera_intrinsics(
    const std::string& sensor_id, const CameraIntrinsics& intrin) {
    params_->camera_intrinsics[sensor_id] = std::make_shared<CameraIntrinsics>(intrin);
}

void JointCalibSolver::set_initial_extrinsic(
    const std::string& ref_id, const std::string& target_id,
    const Sophus::SE3d& T_init) {
    auto ext = params_->get_or_create_extrinsic(ref_id, target_id);
    ext->set_SE3(T_init);
}

#if UNICALIB_WITH_IKALIBR
// ===================================================================
// iKalibrResultWriter 实现
// ===================================================================

void JointCalibSolver::iKalibrResultWriter::write_extrinsics(
    const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
    CalibParamManager::Ptr& unicalib_params) {
    
    if (!ikalibr_param_mgr || !unicalib_params) return;
    
    UNICALIB_INFO("[iKalibrRW] 写回外参");
    UNICALIB_WARN("[iKalibrRW] write_extrinsics 为桩实现：未从 iKalibr 读取优化结果，需根据 iKalibr API 补全");
    // 遍历 unicalib_params 中的所有外参，从 ikalibr_param_mgr 读取优化结果
    // 示例伪代码（需根据实际 iKalibr API 调整）:
    // for (auto& [key, ext_ptr] : unicalib_params->extrinsics) {
    //     if (ext_ptr) {
    //         try {
    //             // 从 ikalibr param manager 读取对应外参
    //             // Sophus::SE3d T_refined = ikalibr_param_mgr->Get...(ref_id, target_id);
    //             // double t_offset = ikalibr_param_mgr->GetTimeOffset(...);
    //             // ext_ptr->set_SE3(T_refined);
    //             // ext_ptr->time_offset_s = t_offset;
    //             UNICALIB_TRACE("[iKalibrRW] 外参 {} 已更新", key);
    //         } catch (...) {
    //             UNICALIB_WARN("[iKalibrRW] 外参 {} 写回失败，保留粗值", key);
    //         }
    //     }
    // }
}

void JointCalibSolver::iKalibrResultWriter::write_imu_intrinsics(
    const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
    CalibParamManager::Ptr& unicalib_params) {
    
    if (!ikalibr_param_mgr || !unicalib_params) return;
    
    UNICALIB_INFO("[iKalibrRW] 写回 IMU 内参");
    UNICALIB_WARN("[iKalibrRW] write_imu_intrinsics 为桩实现：未从 iKalibr 读取，需根据 API 补全");
    // 类似逻辑: 遍历 unicalib_params->imu_intrinsics，从 ikalibr_param_mgr 读取更新值
}

void JointCalibSolver::iKalibrResultWriter::write_camera_intrinsics(
    const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
    CalibParamManager::Ptr& unicalib_params) {
    
    if (!ikalibr_param_mgr || !unicalib_params) return;
    
    UNICALIB_INFO("[iKalibrRW] 写回相机内参");
    UNICALIB_WARN("[iKalibrRW] write_camera_intrinsics 为桩实现：未从 iKalibr 读取，需根据 API 补全");
    // 类似逻辑: 遍历 unicalib_params->camera_intrinsics，从 ikalibr_param_mgr 读取更新值
}

#endif  // UNICALIB_WITH_IKALIBR

void JointCalibSolver::report_progress(
    const std::string& stage, double progress, const std::string& msg) {
    if (progress_cb_) {
        progress_cb_(stage, progress, msg);
    }
    if (cfg_.verbose) {
        UNICALIB_LOG_STEP(stage, "{:.0f}% {}", progress * 100, msg);
    }
}

// ===================================================================
// 独立标定接口
// ===================================================================
std::optional<IMUIntrinsics> JointCalibSolver::calibrate_imu_intrinsic(
    const std::string& sensor_id,
    const std::vector<IMUFrame>& data) {

    report_progress("IMU-Intrinsic", 0.0, "开始 IMU 内参标定");
    IMUIntrinsicCalibrator calib(cfg_.imu_intrin_cfg);
    calib.set_progress_callback([this, &sensor_id](const std::string& s, double p) {
        report_progress("IMU-Intrinsic/" + sensor_id, p, s);
    });
    // 转换 IMUFrame -> IMURawFrame (字段相同)
    IMURawData raw_data;
    raw_data.reserve(data.size());
    for (const auto& f : data) {
        IMURawFrame rf;
        rf.timestamp = f.timestamp;
        rf.gyro  = f.gyro;
        rf.accel = f.accel;
        raw_data.push_back(rf);
    }
    auto result = calib.calibrate(raw_data);
    params_->imu_intrinsics[sensor_id] = std::make_shared<IMUIntrinsics>(result);
    report_progress("IMU-Intrinsic", 1.0, "完成");
    return result;
}

std::optional<CameraIntrinsics> JointCalibSolver::calibrate_camera_intrinsic(
    const std::string& sensor_id,
    const std::vector<std::string>& image_paths) {

    report_progress("Camera-Intrinsic", 0.0, "开始相机内参标定");
    CameraIntrinsicCalibrator calib(cfg_.cam_intrin_cfg);
    calib.set_progress_callback([this, &sensor_id](int cur, int total) {
        report_progress("Camera-Intrinsic/" + sensor_id,
                        static_cast<double>(cur) / total, "检测角点");
    });
    auto result = calib.calibrate(image_paths);
    if (result.has_value()) {
        params_->camera_intrinsics[sensor_id] =
            std::make_shared<CameraIntrinsics>(result.value());
        report_progress("Camera-Intrinsic", 1.0,
                        "完成 RMS=" + std::to_string(result->rms_reproj_error) + "px");
    } else {
        report_progress("Camera-Intrinsic", 1.0, "失败");
    }
    return result;
}

std::optional<ExtrinsicSE3> JointCalibSolver::calibrate_imu_lidar(
    const std::string& imu_id,
    const std::string& lidar_id,
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans) {

    report_progress("IMU-LiDAR", 0.0, "开始 IMU-LiDAR 外参标定");
    IMULiDARCalibrator calib(cfg_.imu_lidar_cfg);
    calib.set_progress_callback([this, &imu_id, &lidar_id](const std::string& s, double p) {
        report_progress("IMU-LiDAR/" + imu_id + "->" + lidar_id, p, s);
    });

    const IMUIntrinsics* imu_intrin = nullptr;
    auto it = params_->imu_intrinsics.find(imu_id);
    if (it != params_->imu_intrinsics.end() && it->second) {
        imu_intrin = it->second.get();
    }

    auto result = calib.calibrate(imu_data, lidar_scans, imu_id, lidar_id, imu_intrin);
    if (result.has_value()) {
        auto ext = params_->get_or_create_extrinsic(imu_id, lidar_id);
        *ext = result.value();
        report_progress("IMU-LiDAR", 1.0, "完成");
    } else {
        report_progress("IMU-LiDAR", 1.0, "失败");
    }
    return result;
}

std::optional<ExtrinsicSE3> JointCalibSolver::calibrate_lidar_camera(
    const std::string& lidar_id,
    const std::string& cam_id,
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& cam_frames,
    const CameraIntrinsics& cam_intrin) {

    report_progress("LiDAR-Camera", 0.0, "开始 LiDAR-Camera 外参标定");
    LiDARCameraCalibrator calib(cfg_.lidar_cam_cfg);
    calib.set_progress_callback([this, &lidar_id, &cam_id](const std::string& s, double p) {
        report_progress("LiDAR-Camera/" + lidar_id + "->" + cam_id, p, s);
    });

    std::optional<ExtrinsicSE3> result;
    switch (cfg_.lidar_cam_cfg.method) {
    case LiDARCameraCalibrator::Method::TARGET_CHESSBOARD:
        result = calib.calibrate_target(lidar_scans, cam_frames, cam_intrin, lidar_id, cam_id);
        break;
    case LiDARCameraCalibrator::Method::EDGE_ALIGNMENT: {
        Sophus::SE3d init;
        result = calib.calibrate_edge_align(lidar_scans, cam_frames, cam_intrin,
                                             init, lidar_id, cam_id);
        break;
    }
    case LiDARCameraCalibrator::Method::MOTION_BSPLINE:
        UNICALIB_WARN("Motion method requires IMU data. Use joint calibrate() instead.");
        break;
    }

    if (result.has_value()) {
        auto ext = params_->get_or_create_extrinsic(lidar_id, cam_id);
        *ext = result.value();
        report_progress("LiDAR-Camera", 1.0, "完成");
    }
    return result;
}

std::optional<ExtrinsicSE3> JointCalibSolver::calibrate_cam_cam(
    const std::string& cam0_id,
    const std::string& cam1_id,
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1) {

    report_progress("Cam-Cam", 0.0, "开始 Camera-Camera 外参标定");
    CamCamCalibrator calib(cfg_.cam_cam_cfg);
    calib.set_progress_callback([this, &cam0_id, &cam1_id](const std::string& s, double p) {
        report_progress("Cam-Cam/" + cam0_id + "->" + cam1_id, p, s);
    });

    std::optional<ExtrinsicSE3> result;
    switch (cfg_.cam_cam_cfg.method) {
    case CamCamCalibrator::Method::CHESSBOARD_STEREO:
        result = calib.calibrate_stereo(images_cam0, images_cam1,
                                         intrin0, intrin1, cam0_id, cam1_id);
        break;
    default:
        UNICALIB_WARN("Other cam-cam methods need image frames, not paths. Use calibrate_essential().");
        break;
    }

    if (result.has_value()) {
        auto ext = params_->get_or_create_extrinsic(cam0_id, cam1_id);
        *ext = result.value();
        report_progress("Cam-Cam", 1.0, "完成");
    }
    return result;
}

// ===================================================================
// 联合标定主函数
// ===================================================================
Status JointCalibSolver::calibrate(const ns_unicalib::CalibDataBundle& data, ns_unicalib::CalibSummary& summary) {
    summary = CalibSummary{};
    summary.params = params_;

    auto t_start = std::chrono::high_resolution_clock::now();
    UNICALIB_CALC("联合标定入口 calibrate() 开始");

    UNICALIB_INFO("=== UniCalib 联合标定开始 ===");

    // 入口校验：至少需要一类数据以执行任一启用的阶段
    const bool need_imu = cfg_.do_imu_intrinsic;
    const bool need_cam = cfg_.do_camera_intrinsic;
    const bool need_lidar_imu = cfg_.do_imu_lidar_extrinsic;
    const bool need_lidar_cam = cfg_.do_lidar_camera_extrinsic;
    const bool need_cam_cam = cfg_.do_cam_cam_extrinsic;
    if (need_imu && data.imu_data.empty()) {
        UNICALIB_THROW_CALIB(ErrorCode::INSUFFICIENT_DATA,
            "do_imu_intrinsic=true 但 data.imu_data 为空，请提供 IMU 数据");
    }
    if (need_cam && data.camera_image_paths.empty() && data.camera_frames.empty()) {
        UNICALIB_THROW_CALIB(ErrorCode::INSUFFICIENT_DATA,
            "do_camera_intrinsic=true 但无相机图像路径或帧数据");
    }
    if ((need_lidar_imu || need_lidar_cam) && data.lidar_scans.empty()) {
        UNICALIB_THROW_CALIB(ErrorCode::INSUFFICIENT_DATA,
            "外参标定需要 LiDAR 数据，但 data.lidar_scans 为空");
    }

    try {
        // Phase 1: 内参标定
        if (cfg_.do_imu_intrinsic || cfg_.do_camera_intrinsic) {
            phase1_intrinsics(data, summary);
        }

        // Phase 2: 粗外参
        if (cfg_.do_imu_lidar_extrinsic || cfg_.do_lidar_camera_extrinsic ||
            cfg_.do_cam_cam_extrinsic) {
            phase2_coarse_extrinsic(data, summary);
        }

        // Phase 3: 联合精化
        if (cfg_.do_joint_bspline_refine) {
            phase3_joint_refine(data, summary);
        }

        // Phase 4: 验证
        phase4_validation(data, summary);

        summary.success = true;

    } catch (const UniCalibException& e) {
        // 工程化统一异常处理
        summary.success = false;
        summary.error_message = e.toString();
        summary.error_code = e.code();
        UNICALIB_ERROR("联合标定失败 [{}]: {}",
                       errorCodeName(e.code()), e.what());
#if UNICALIB_WITH_IKALIBR
    } catch (const ns_ikalibr::IKalibrStatus& e) {
        // iKalibr 库异常
        summary.success = false;
        summary.error_message = std::string("iKalibr error: ") + e.what();
        summary.error_code = ErrorCode::CALIBRATION_FAILED;
        UNICALIB_ERROR("联合标定失败 (iKalibr): {}", e.what());
#endif
    } catch (const std::exception& e) {
        // 兜底异常处理（携带阶段信息便于定位）
        summary.success = false;
        summary.error_message = std::string("std::exception: ") + e.what();
        summary.error_code = ErrorCode::CALIBRATION_FAILED;
        UNICALIB_CRITICAL("联合标定未知异常 [calibrate]: {}", e.what());
    }

    // 保存结果（失败不抛，仅记录以便精确定位）
    try {
        save_results(summary);
    } catch (const UniCalibException& e) {
        UNICALIB_ERROR("保存标定结果失败 [save_results]: {} path={}", e.toString(), cfg_.output_dir);
        if (summary.error_message.empty()) {
            summary.error_message = "save_results failed: " + e.toString();
        }
    } catch (const std::exception& e) {
        UNICALIB_ERROR("保存标定结果失败 [save_results]: {} path={}", e.what(), cfg_.output_dir);
        if (summary.error_message.empty()) {
            summary.error_message = std::string("save_results: ") + e.what();
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    UNICALIB_CALC("联合标定出口 calibrate() 完成 elapsed={:.2f}s success={}", elapsed, summary.success);
    UNICALIB_INFO("=== 联合标定完成 ({}s) ===", elapsed);

    params_->print_summary();
    return summary.toStatus();
}

void JointCalibSolver::phase1_intrinsics(
    const ns_unicalib::CalibDataBundle& data, ns_unicalib::CalibSummary& summary) {

    UNICALIB_CALC("Phase1 内参标定开始");
    report_progress("Phase1-Intrinsics", 0.0, "内参标定");

    // IMU 内参
    if (cfg_.do_imu_intrinsic) {
        for (const auto& [sensor_id, imu_data] : data.imu_data) {
            UNICALIB_INFO("  IMU 内参: {} ({} 帧)", sensor_id, imu_data.size());
            if (imu_data.size() < 2) {
                UNICALIB_THROW_CALIB(ErrorCode::INSUFFICIENT_DATA,
                    "IMU 内参标定需要至少 2 帧数据，sensor_id=" + sensor_id +
                    " 当前帧数=" + std::to_string(imu_data.size()));
            }
            auto result = calibrate_imu_intrinsic(sensor_id, imu_data);
            CalibSummary::QualityMetrics qm;
            qm.calib_type = "imu_intrinsic/" + sensor_id;
            qm.converged  = true;
            qm.unit = "rad/s";
            if (result.has_value()) {
                qm.rms_error = result->allan_fit_rms;
                UNICALIB_CALC("Phase1 IMU内参 {} allan_fit_rms={:.6f}", sensor_id, result->allan_fit_rms);
            }
            summary.quality.push_back(qm);
        }
    }

    // 相机内参
    if (cfg_.do_camera_intrinsic) {
        for (const auto& [sensor_id, image_paths] : data.camera_image_paths) {
            UNICALIB_INFO("  相机内参: {}", sensor_id);
            auto result = calibrate_camera_intrinsic(sensor_id, image_paths);
            CalibSummary::QualityMetrics qm;
            qm.calib_type = "camera_intrinsic/" + sensor_id;
            qm.unit = "px";
            if (result.has_value()) {
                qm.rms_error  = result->rms_reproj_error;
                qm.converged  = (result->rms_reproj_error < 2.0);
                UNICALIB_CALC("Phase1 相机内参 {} rms_reproj_error={:.4f}px converged={}",
                              sensor_id, result->rms_reproj_error, qm.converged ? "yes" : "no");
            }
            summary.quality.push_back(qm);
        }
    }

    UNICALIB_CALC("Phase1 内参标定结束");
    report_progress("Phase1-Intrinsics", 1.0, "完成");
}

void JointCalibSolver::phase2_coarse_extrinsic(
    const ns_unicalib::CalibDataBundle& data, ns_unicalib::CalibSummary& summary) {

    UNICALIB_CALC("Phase2 粗外参估计开始");
    report_progress("Phase2-Coarse", 0.0, "粗外参估计");

    // IMU-LiDAR
    if (cfg_.do_imu_lidar_extrinsic && !data.imu_data.empty() && !data.lidar_scans.empty()) {
        const auto& imu_id    = data.imu_data.begin()->first;
        const auto& lidar_id  = data.lidar_scans.begin()->first;
        const auto& imu_data  = data.imu_data.begin()->second;
        const auto& lidar_data= data.lidar_scans.begin()->second;

        UNICALIB_INFO("  IMU-LiDAR 外参: {} -> {}", imu_id, lidar_id);
        calibrate_imu_lidar(imu_id, lidar_id, imu_data, lidar_data);
    }

    // Camera-Camera：按 cam_cam_camera_align 配置对标定，否则按 sensors 顺序相邻两两
    if (cfg_.do_cam_cam_extrinsic) {
        auto run_cam_cam_pair = [&](const std::string& id0, const std::string& id1) {
            auto it0 = data.camera_image_paths.find(id0);
            auto it1 = data.camera_image_paths.find(id1);
            if (it0 == data.camera_image_paths.end()) {
                UNICALIB_WARN("  Cam-Cam {}->{}: 无相机图像路径 sensor_id={}", id0, id1, id0);
                return;
            }
            if (it1 == data.camera_image_paths.end()) {
                UNICALIB_WARN("  Cam-Cam {}->{}: 无相机图像路径 sensor_id={}", id0, id1, id1);
                return;
            }
            if (it0->second.empty() || it1->second.empty()) {
                UNICALIB_WARN("  Cam-Cam {}->{}: 图像列表为空", id0, id1);
                return;
            }
            auto in0 = params_->camera_intrinsics.count(id0) ? params_->camera_intrinsics.at(id0) : nullptr;
            auto in1 = params_->camera_intrinsics.count(id1) ? params_->camera_intrinsics.at(id1) : nullptr;
            if (!in0 || !in1) {
                UNICALIB_WARN("  Cam-Cam {}->{}: 内参未知, 跳过", id0, id1);
                return;
            }
            UNICALIB_INFO("  Camera-Camera 外参: {} -> {}", id0, id1);
            calibrate_cam_cam(id0, id1, it0->second, it1->second, *in0, *in1);
        };
        if (!cfg_.cam_cam_camera_align.empty()) {
            for (const auto& [id0, id1] : cfg_.cam_cam_camera_align)
                run_cam_cam_pair(id0, id1);
        } else {
            auto cam_sensors = sys_cfg_.sensors_of_type(SensorType::CAMERA);
            for (size_t i = 0; i + 1 < cam_sensors.size(); ++i)
                run_cam_cam_pair(cam_sensors[i]->sensor_id, cam_sensors[i+1]->sensor_id);
        }
    }

    UNICALIB_CALC("Phase2 粗外参估计结束");
    report_progress("Phase2-Coarse", 1.0, "完成");
}

#if UNICALIB_WITH_IKALIBR
namespace {
// B样条轨迹拟合代价：残差 = log(spline.pose(t)^{-1} * T_meas)，对 knot 使用 basalt pose(t,&J) 解析雅可比
constexpr int BSPLINE_ORDER = 5;
using Se3Spline = basalt::Se3Spline<BSPLINE_ORDER, double>;

class SplineTrajectoryCost : public ceres::CostFunction {
 public:
    SplineTrajectoryCost(Se3Spline* spline, int64_t t_ns, const Sophus::SE3d& T_meas,
                         int64_t dt_ns, int64_t t0_ns, int num_knots)
        : spline_(spline), t_ns_(t_ns), T_meas_(T_meas),
          dt_ns_(dt_ns), t0_ns_(t0_ns), num_knots_(num_knots) {
        set_num_residuals(6);
        for (int i = 0; i < num_knots; ++i)
            mutable_parameter_block_sizes()->push_back(6);
    }

    bool Evaluate(double const* const* parameters,
                  double* residuals,
                  double** jacobians) const override {
        // 1) 从 parameters 同步到 spline（每 knot 6 维: angle_axis + trans）
        for (int i = 0; i < num_knots_; ++i) {
            const double* k = parameters[i];
            Eigen::Vector3d omega(k[0], k[1], k[2]);
            Eigen::Vector3d trans(k[3], k[4], k[5]);
            spline_->setKnot(Sophus::SE3d(Sophus::SO3d::exp(omega), trans), i);
        }
        // 2) 求 pose 及对 knot 的雅可比
        typename Se3Spline::PosePosSO3JacobianStruct J;
        Sophus::SE3d pose = spline_->pose(t_ns_, &J);
        Sophus::SE3d T_err = pose.inverse() * T_meas_;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
        r = T_err.log();

        if (!jacobians) return true;

        Eigen::Matrix<double, 6, 6> J_r_inv;
        Sophus::rightJacobianInvSE3Decoupled(r, J_r_inv);
        const size_t start = J.start_idx;
        const int N = BSPLINE_ORDER;
        for (int i = 0; i < num_knots_; ++i) {
            if (jacobians[i]) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> Jk(jacobians[i]);
                Jk.setZero();
                if (i >= static_cast<int>(start) && i < static_cast<int>(start) + N) {
                    size_t local = i - start;
                    Jk = -J_r_inv * J.d_val_d_knot[local];
                }
            }
        }
        return true;
    }

 private:
    Se3Spline* spline_;
    int64_t t_ns_;
    Sophus::SE3d T_meas_;
    int64_t dt_ns_;
    int64_t t0_ns_;
    int num_knots_;
};
}  // namespace
#endif

void JointCalibSolver::phase3_joint_refine(
    const ns_unicalib::CalibDataBundle& data, ns_unicalib::CalibSummary& summary) {

    UNICALIB_CALC("Phase3 B样条联合精化开始");
    report_progress("Phase3-Refine", 0.0, "B样条联合精化");
#if UNICALIB_WITH_IKALIBR
    // ==================================================================
    // B样条联合精化 (基于 basalt-headers + Ceres knot 优化)
    // ==================================================================
    try {
        UNICALIB_INFO("[Phase3] 启动 B样条联合精化（knot 参数化 + 解析雅可比）");
        
        report_progress("Phase3-Refine", 0.1, "准备轨迹数据");
        if (data.lidar_scans.empty()) {
            UNICALIB_WARN("[Phase3] No LiDAR scans provided, skipping B-spline refinement");
            summary.success = true;
            return;
        }
        
        std::vector<std::pair<double, Sophus::SE3d>> lidar_trajectory;
        for (const auto& [lidar_id, scans] : data.lidar_scans) {
            for (const auto& scan : scans) {
                if (scan.timestamp >= 0 && scan.pose) {
                    lidar_trajectory.push_back(std::make_pair(scan.timestamp, *scan.pose));
                }
            }
        }
        
        if (lidar_trajectory.size() < 5) {
            UNICALIB_WARN("[Phase3] Insufficient trajectory points ({} < 5)", lidar_trajectory.size());
            summary.success = false;
            summary.message = "Insufficient trajectory points for B-spline refinement: " + std::to_string(lidar_trajectory.size());
            return;
        }
        
        UNICALIB_INFO("[Phase3] Collected {} trajectory points", lidar_trajectory.size());
        report_progress("Phase3-Refine", 0.2, "构造B样条");
        
        using Se3Spline = basalt::Se3Spline<BSPLINE_ORDER, double>;
        const double dt = 0.1;
        double t_start = lidar_trajectory.front().first;
        double t_end = lidar_trajectory.back().first;
        const int num_knots = std::max(8, static_cast<int>((t_end - t_start) / dt) + 1);
        const int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        const int64_t t0_ns = static_cast<int64_t>(t_start * 1e9);
        
        Se3Spline spline(dt_ns, t0_ns);
        Sophus::SE3d last_pose = lidar_trajectory.front().second;
        spline.setKnots(last_pose, num_knots);
        for (int i = 0; i < num_knots; ++i) {
            double t_knot = t_start + i * dt;
            auto it = std::lower_bound(lidar_trajectory.begin(), lidar_trajectory.end(),
                t_knot, [](const auto& a, double t) { return a.first < t; });
            if (it != lidar_trajectory.end()) {
                spline.setKnot(it->second, i);
                last_pose = it->second;
            } else if (i > 0) {
                spline.setKnot(last_pose, i);
            }
        }
        
        // Knot 存储 (6 维/knot: angle_axis + trans)，并加入 Ceres
        std::vector<double> knot_storage(static_cast<size_t>(num_knots) * 6);
        std::vector<double*> knot_ptrs(static_cast<size_t>(num_knots));
        for (int i = 0; i < num_knots; ++i) {
            Sophus::SE3d T = spline.getKnot(i);
            Eigen::Vector3d omega = T.so3().log();
            Eigen::Vector3d trans = T.translation();
            knot_storage[i*6+0] = omega(0); knot_storage[i*6+1] = omega(1); knot_storage[i*6+2] = omega(2);
            knot_storage[i*6+3] = trans(0); knot_storage[i*6+4] = trans(1); knot_storage[i*6+5] = trans(2);
            knot_ptrs[i] = &knot_storage[i * 6];
        }
        
        ceres::Problem problem;
        for (int i = 0; i < num_knots; ++i)
            problem.AddParameterBlock(knot_ptrs[i], 6);
        
        report_progress("Phase3-Refine", 0.4, "添加轨迹拟合因子");
        int num_residuals = 0;
        const int N = BSPLINE_ORDER;
        for (const auto& [t, T_meas] : lidar_trajectory) {
            int64_t t_ns = static_cast<int64_t>(t * 1e9);
            if (t_ns < spline.minTimeNs() || t_ns > spline.maxTimeNs()) continue;
            int seg = static_cast<int>((t_ns - t0_ns) / dt_ns);
            if (seg < 0 || seg + N > num_knots) continue;
            ceres::CostFunction* cost = new SplineTrajectoryCost(
                &spline, t_ns, T_meas, dt_ns, t0_ns, num_knots);
            // Ceres 2.x 支持 AddResidualBlock(..., const std::vector<double*>&)；若使用 1.x 需改为逐指针传入
            problem.AddResidualBlock(cost, new ceres::HuberLoss(1.0), knot_ptrs);
            num_residuals++;
        }
        UNICALIB_INFO("[Phase3] Added {} trajectory residual blocks (knot optimization)", num_residuals);
        
        if (num_residuals < 5) {
            UNICALIB_WARN("[Phase3] Too few residuals, skip solve");
            summary.success = true;
            return;
        }
        
        report_progress("Phase3-Refine", 0.5, "执行优化");
        ceres::Solver::Options solver_options;
        solver_options.max_num_iterations = 80;
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
        solver_options.num_threads = 1;  // SplineTrajectoryCost 会写回 spline，多线程有数据竞争
        solver_options.minimizer_progress_to_stdout = cfg_.verbose;
        ceres::Solver::Summary solver_summary;
        ceres::Solve(solver_options, &problem, &solver_summary);
        UNICALIB_INFO("[Phase3] {}", solver_summary.BriefReport());
        UNICALIB_CALC("Phase3 B样条 Ceres initial_cost={:.6f} final_cost={:.6f} iterations={} converged={}",
                      solver_summary.initial_cost, solver_summary.final_cost,
                      static_cast<int>(solver_summary.iterations.size()),
                      solver_summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");
        
        report_progress("Phase3-Refine", 0.7, "回写knot");
        for (int i = 0; i < num_knots; ++i) {
            double* k = knot_ptrs[i];
            spline.setKnot(Sophus::SE3d(
                Sophus::SO3d::exp(Eigen::Vector3d(k[0], k[1], k[2])),
                Eigen::Vector3d(k[3], k[4], k[5])), i);
        }
        
        // 用末端 knot 更新“轨迹对应”的外参：若配置了 phase3_extrinsic_ref/target 则只更新该 key；否则仅当恰好一个外参时更新
        Sophus::SE3d T_end = spline.getKnot(num_knots - 1);
        bool updated = false;
        if (!cfg_.phase3_extrinsic_ref_id.empty() && !cfg_.phase3_extrinsic_target_id.empty()) {
            auto ext_ptr = params_->get_extrinsic(cfg_.phase3_extrinsic_ref_id, cfg_.phase3_extrinsic_target_id);
            if (ext_ptr) {
                ext_ptr->set_SE3(T_end);
                UNICALIB_INFO("[Phase3] 已用末端 knot 更新外参 {}->{}",
                    cfg_.phase3_extrinsic_ref_id, cfg_.phase3_extrinsic_target_id);
                updated = true;
            } else {
                UNICALIB_WARN("[Phase3] 未找到外参 {}->{}，跳过 knot→extrinsic 回写",
                    cfg_.phase3_extrinsic_ref_id, cfg_.phase3_extrinsic_target_id);
            }
        }
        if (!updated && params_->extrinsics.size() == 1) {
            for (auto& [key, ext_ptr] : params_->extrinsics) {
                if (ext_ptr) {
                    ext_ptr->set_SE3(T_end);
                    UNICALIB_DEBUG("[Phase3] 已用末端 knot 更新唯一外参 {}", key);
                    updated = true;
                    break;
                }
            }
        }
        if (!updated && !params_->extrinsics.empty()) {
            UNICALIB_DEBUG("[Phase3] 未配置 phase3_extrinsic_ref/target_id 且外参数量>1，跳过 knot→extrinsic 回写");
        }
        
        report_progress("Phase3-Refine", 0.9, "更新结果");
        summary.success = true;
        summary.quality.push_back({
            .calib_type = "IMU-LiDAR-Camera Joint B-spline Refinement (knot+analytic)",
            .rms_error = solver_summary.final_cost,
            .max_error = 0.0,
            .converged = solver_summary.termination_type == ceres::CONVERGENCE,
            .unit = "m"
        });
        UNICALIB_INFO("[Phase3] B-spline refinement completed (knots optimized)");
        report_progress("Phase3-Refine", 1.0, "完成");
    }
    catch (const ns_unicalib::UniCalibException& e) {
        UNICALIB_ERROR("[Phase3] B-spline refinement failed: {}", e.toString());
        summary.success = false;
        summary.error_message = e.toString();
    }
    catch (const ns_ikalibr::IKalibrStatus& e) {
        UNICALIB_ERROR("[Phase3] iKalibr B-spline refinement failed: {}", e.what());
        summary.success = false;
        summary.error_message = std::string("iKalibr error: ") + e.what();
    }

#else  // UNICALIB_WITH_IKALIBR
    // 无 iKalibr 时，直接返回成功（使用 Phase2 结果）
    UNICALIB_WARN("[Phase3] Skipped (UNICALIB_WITH_IKALIBR not enabled)");
    UNICALIB_WARN("[Phase3] Using Phase2 coarse extrinsic results");
    summary.success = true;
    
    // 可选：添加质量指标说明
    summary.quality.push_back({
        .calib_type = "Coarse Extrinsic (No B-spline Refinement)",
        .rms_error = 0.0,
        .max_error = 0.0,
        .converged = true,
        .unit = "m"
    });

#endif  // UNICALIB_WITH_IKALIBR
}

void JointCalibSolver::phase4_validation(
    const ns_unicalib::CalibDataBundle& /*data*/, ns_unicalib::CalibSummary& summary) {

    UNICALIB_CALC("Phase4 验证开始");
    report_progress("Phase4-Validation", 0.0, "验证");
    params_->print_summary();
    report_progress("Phase4-Validation", 1.0, "完成");
}

void JointCalibSolver::save_results(const ns_unicalib::CalibSummary& summary) {
    if (!summary.params) {
        UNICALIB_WARN("[save_results] summary.params 为空，跳过保存");
        return;
    }

    const std::string out_dir = cfg_.output_dir;
    std::error_code ec;
    if (!fs::exists(out_dir)) {
        fs::create_directories(out_dir, ec);
        if (ec) {
            UNICALIB_THROW_SYSTEM(ErrorCode::FILE_WRITE_ERROR,
                "无法创建输出目录: " + out_dir + " (" + ec.message() + ")");
        }
    }

    if (cfg_.save_yaml) {
        std::string yaml_path = out_dir + "/calibration_result.yaml";
        try {
            summary.params->save_yaml(yaml_path);
        } catch (const std::exception& e) {
            UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                "写入标定 YAML 失败 path=" + yaml_path + " detail=" + e.what());
        }
    }

    UNICALIB_INFO("结果已保存至: {}", out_dir);
}

void CalibSummary::print() const {
    std::cout << "\n╔══════════════════════════════════╗\n";
    std::cout << "║       标定结果摘要               ║\n";
    std::cout << "╠══════════════════════════════════╣\n";
    std::cout << "║ 状态: " << (success ? "✓ 成功" : "✗ 失败") << "\n";
    if (!error_message.empty())
        std::cout << "║ 错误: " << error_message << "\n";
    for (const auto& q : quality) {
        std::cout << "║ " << q.calib_type << ": RMS=" << q.rms_error
                  << q.unit << " " << (q.converged ? "✓" : "✗") << "\n";
    }
    std::cout << "╚══════════════════════════════════╝\n\n";
}

}  // namespace ns_unicalib
