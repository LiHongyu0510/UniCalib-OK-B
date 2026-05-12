#pragma once
/**
 * UniCalib Unified — 联合标定求解器 (Orchestrator)
 * 协调五种标定的执行顺序，支持独立标定和联合标定
 *
 * 执行顺序 (联合模式):
 *   Phase 1: 内参标定
 *     1a. IMU 内参 (Allan + 六面法)
 *     1b. 所有相机内参 (棋盘格)
 *   Phase 2: 粗外参初始化
 *     2a. IMU-LiDAR 旋转 (手眼)
 *     2b. LiDAR-Camera 初始化 (PnP 或单应)
 *     2c. Camera-Camera 初始化 (本质矩阵)
 *   Phase 3: 精细优化
 *     3. B样条联合时空优化 (iKalibr 核心)
 *   Phase 4: 验证
 *     4a. 重投影误差评估
 *     4b. 边缘对齐评分
 *     4c. HTML 报告生成
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/common/status.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/timing.h"
#include "unicalib/common/health_monitor.h"
#include "unicalib/common/diagnostics.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/intrinsic/camera_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <any>
#include <fmt/core.h>
#include <fmt/format.h>

namespace ns_unicalib {

// ===================================================================
// 数据加载接口 (抽象)
// ===================================================================
struct CalibDataBundle {
    // 按传感器 ID 存储的数据
    std::map<std::string, std::vector<IMUFrame>>   imu_data;
    std::map<std::string, std::vector<LiDARScan>>  lidar_scans;
    std::map<std::string, std::vector<std::pair<double, cv::Mat>>> camera_frames;
    // 图像路径 (懒加载)
    std::map<std::string, std::vector<std::string>> camera_image_paths;

    bool empty() const {
        return imu_data.empty() && lidar_scans.empty() && camera_frames.empty();
    }
};

// ===================================================================
// 标定结果汇总
// ===================================================================
struct CalibSummary {
    bool success = false;
    ErrorCode error_code = ErrorCode::SUCCESS;
    std::string error_message;
    CalibParamManager::Ptr params;

    // 每种标定的质量指标
    struct QualityMetrics {
        std::string calib_type;
        double rms_error = 0.0;
        double max_error = 0.0;
        bool   converged = false;
        std::string unit;   // "px", "m", "deg", etc.
        size_t num_samples = 0;
        double duration_ms = 0.0;  // 执行耗时
    };
    std::vector<QualityMetrics> quality;

    // 整体性能统计
    double total_duration_ms = 0.0;
    std::map<std::string, double> phase_durations_ms;

    // 输出文件路径
    std::string yaml_path;
    std::string json_path;
    std::string report_html_path;
    std::string diagnostics_path;

    // 健康状态
    HealthStatus overall_health = HealthStatus::HEALTHY;
    std::map<std::string, HealthStatus> module_health;

    // 打印摘要
    void print() const;
    
    // 转换为 Status
    Status toStatus() const {
        if (success) {
            return Status::OK();
        } else {
            return Status::Error(error_code, error_message);
        }
    }
};

// ===================================================================
// 联合标定求解器
// ===================================================================
class JointCalibSolver {
public:
    struct Config {
        // 标定选择标志
        bool do_imu_intrinsic        = true;
        bool do_camera_intrinsic     = true;
        bool do_imu_lidar_extrinsic   = true;
        bool do_lidar_lidar_extrinsic = false;
        bool do_lidar_camera_extrinsic = true;
        bool do_cam_cam_extrinsic    = true;

        // 是否执行 iKalibr 联合 B样条精化 (需要足够数据)
        bool do_joint_bspline_refine = true;
        /** Phase3 轨迹对应的外参：仅更新此前缀的外参（避免多外参时误覆盖）。
         *  若均非空则只更新 get_extrinsic(phase3_extrinsic_ref_id, phase3_extrinsic_target_id)；
         *  若均为空且当前仅 1 个外参则更新该唯一外参；否则不写回。 */
        std::string phase3_extrinsic_ref_id;
        std::string phase3_extrinsic_target_id;

        // IMU 内参配置
        IMUIntrinsicCalibrator::Config imu_intrin_cfg;
        // 相机内参配置
        CameraIntrinsicCalibrator::Config cam_intrin_cfg;
        // IMU-LiDAR 外参配置
        IMULiDARCalibrator::Config imu_lidar_cfg;
        // LiDAR-Camera 外参配置
        LiDARCameraCalibrator::Config lidar_cam_cfg;
        // Camera-Camera 外参配置
        CamCamCalibrator::Config cam_cam_cfg;
        // Camera-Camera 标定对：[(id0, id1), ...]，非空时按此列表标定；空则按 sensors 顺序两两
        std::vector<std::pair<std::string, std::string>> cam_cam_camera_align;

        // 输出配置
        std::string output_dir = "./results";
        bool save_yaml = true;
        bool save_json = true;
        bool generate_html_report = true;
        bool verbose = true;
    };

    explicit JointCalibSolver(const Config& cfg);
    JointCalibSolver();

    // 设置系统配置 (传感器列表 + 标定对)
    void set_system_config(const SystemConfig& sys_cfg);

    // 设置已知内参 (跳过内参标定步骤)
    void set_imu_intrinsics(const std::string& sensor_id,
                            const IMUIntrinsics& intrin);
    void set_camera_intrinsics(const std::string& sensor_id,
                               const CameraIntrinsics& intrin);

    // 设置初始外参猜测
    void set_initial_extrinsic(const std::string& ref_id,
                               const std::string& target_id,
                               const Sophus::SE3d& T_init);

    // 主标定函数 (使用新的Status返回)
    Status calibrate(const CalibDataBundle& data, CalibSummary& summary);

    // 独立标定接口
    std::optional<IMUIntrinsics> calibrate_imu_intrinsic(
        const std::string& sensor_id,
        const std::vector<IMUFrame>& data);

    std::optional<CameraIntrinsics> calibrate_camera_intrinsic(
        const std::string& sensor_id,
        const std::vector<std::string>& image_paths);

    std::optional<ExtrinsicSE3> calibrate_imu_lidar(
        const std::string& imu_id,
        const std::string& lidar_id,
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans);

    std::optional<ExtrinsicSE3> calibrate_lidar_camera(
        const std::string& lidar_id,
        const std::string& cam_id,
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& cam_frames,
        const CameraIntrinsics& cam_intrin);

    std::optional<ExtrinsicSE3> calibrate_cam_cam(
        const std::string& cam0_id,
        const std::string& cam1_id,
        const std::vector<std::string>& images_cam0,
        const std::vector<std::string>& images_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1);

    // 进度回调 (0.0 - 1.0)
    using ProgressCallback = std::function<void(
        const std::string& stage,
        double progress,
        const std::string& message)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = cb; }

    // 获取当前标定参数 (可在标定过程中查询)
    CalibParamManager::Ptr get_params() const { return params_; }

#if UNICALIB_WITH_IKALIBR
    // Phase3 辅助: iKalibr 参数回写
    // 用于将 iKalibr 优化结果写回 params_
    struct iKalibrResultWriter {
        void write_extrinsics(
            const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
            CalibParamManager::Ptr& unicalib_params);
        
        void write_imu_intrinsics(
            const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
            CalibParamManager::Ptr& unicalib_params);
        
        void write_camera_intrinsics(
            const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
            CalibParamManager::Ptr& unicalib_params);
    };
#endif  // UNICALIB_WITH_IKALIBR

private:
    Config cfg_;
    SystemConfig sys_cfg_;
    CalibParamManager::Ptr params_;
    ProgressCallback progress_cb_;

    // 内部执行各阶段
    void phase1_intrinsics(const CalibDataBundle& data, CalibSummary& summary);
    void phase2_coarse_extrinsic(const CalibDataBundle& data, CalibSummary& summary);
    void phase3_joint_refine(const CalibDataBundle& data, CalibSummary& summary);
    void phase4_validation(const CalibDataBundle& data, CalibSummary& summary);

    // 生成输出文件
    void save_results(const CalibSummary& summary);

    void report_progress(const std::string& stage, double progress,
                         const std::string& msg = "");
};

}  // namespace ns_unicalib
