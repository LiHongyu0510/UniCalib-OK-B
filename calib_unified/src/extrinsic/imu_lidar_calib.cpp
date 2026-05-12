/**
 * UniCalib Unified — IMU-LiDAR 外参标定实现
 *
 * 两阶段标定流程:
 *   Stage 1 粗标定: LiDAR 里程计 + 手眼标定
 *   Stage 2 精标定: B样条连续时间优化 (SE3 轨迹 + 外参 + 时间偏移)
 */

#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/math_safety.h"
#include "ikalibr/core/lidar_odometer.h"
#include "ikalibr/sensor/lidar.h"
#include "ikalibr/util/cloud_define.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <numeric>
#include <ceres/ceres.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/utils/sophus_utils.hpp>
#include <cmath>
#include <random>

namespace ns_unicalib {

namespace {
// RANSAC 手眼辅助：从指定下标旋转对构建约束矩阵 M，解出 X，或计算全量残差
Eigen::MatrixXd buildHandeyeM(const std::vector<LiDARRotPair>& pairs,
                              const std::vector<size_t>& indices,
                              bool use_quality_weight) {
    const int n = static_cast<int>(indices.size());
    Eigen::MatrixXd M(4 * n, 4);
    M.setZero();
    for (int ii = 0; ii < n; ++ii) {
        const auto& p = pairs[indices[ii]];
        Eigen::Quaterniond q_A = p.rot_IMU.unit_quaternion();
        Eigen::Quaterniond q_B = p.rot_LiDAR.unit_quaternion();
        if (q_A.w() < 0) q_A.coeffs() = -q_A.coeffs();
        if (q_B.w() < 0) q_B.coeffs() = -q_B.coeffs();
        double sqrt_w = use_quality_weight ? std::sqrt(std::max(p.quality, 0.001)) : 1.0;
        double row[4][4];
        row[0][0] = q_A.w() - q_B.w(); row[0][1] = -q_A.x() - q_B.x(); row[0][2] = -q_A.y() - q_B.y(); row[0][3] = -q_A.z() - q_B.z();
        row[1][0] = q_A.x() + q_B.x(); row[1][1] = q_A.w() + q_B.w(); row[1][2] = q_A.z() - q_B.z(); row[1][3] = -q_A.y() - q_B.y();
        row[2][0] = q_A.y() + q_B.y(); row[2][1] = q_A.y() - q_B.z(); row[2][2] = q_A.w() + q_B.w(); row[2][3] = -q_A.z() - q_B.x();
        row[3][0] = q_A.z() + q_B.z(); row[3][1] = q_A.z() + q_B.y(); row[3][2] = q_A.y() + q_B.x(); row[3][3] = q_A.w() + q_B.w();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                M(4 * ii + r, c) = row[r][c] * sqrt_w;
    }
    return M;
}

std::optional<Sophus::SO3d> solveHandeyeFromM(const Eigen::MatrixXd& M) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    Eigen::Vector4d q_vec = svd.matrixV().col(3);
    double q_norm = q_vec.norm();
    if (q_norm < 1e-10) return std::nullopt;
    q_vec /= q_norm;
    if (q_vec(3) < 0) q_vec = -q_vec;
    Eigen::Quaterniond q(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    return Sophus::SO3d(q);
}

std::vector<double> handeyeResidualsDeg(const std::vector<LiDARRotPair>& pairs, const Sophus::SO3d& X) {
    std::vector<double> out;
    out.reserve(pairs.size());
    for (const auto& p : pairs) {
        Sophus::SO3d lhs = p.rot_IMU * X;
        Sophus::SO3d rhs = X * p.rot_LiDAR;
        double e_rad = (lhs * rhs.inverse()).log().norm();
        out.push_back(MathSafety::radToDeg(e_rad));
    }
    return out;
}
}  // namespace

using namespace ns_ikalibr;
using namespace ns_ctraj;

// 构造函数/析构函数已在头文件中 inline 定义，此处不再重复定义

// ===================================================================
// 主标定函数
// ===================================================================

std::optional<ExtrinsicSE3> IMULiDARCalibrator::calibrate(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin,
    const std::optional<Sophus::SE3d>& coarse_init) {

    if (imu_data.empty()) {
        UNICALIB_ERROR("[IMU-LiDAR] IMU 数据为空");
        return std::nullopt;
    }

    if (lidar_scans.empty()) {
        UNICALIB_ERROR("[IMU-LiDAR] LiDAR 数据为空");
        return std::nullopt;
    }

    UNICALIB_INFO("[IMU-LiDAR] 开始两阶段标定");
    UNICALIB_INFO("  IMU 数据: {} 帧", imu_data.size());
    UNICALIB_INFO("  LiDAR 数据: {} 帧", lidar_scans.size());

    // ---------- 时间戳对齐诊断（标定要求 IMU 与 LiDAR 同一时间基准，未对齐会导致旋转对错误、约 180° 等偏差）----------
    const double imu_t_min = imu_data.front().timestamp;
    const double imu_t_max = imu_data.back().timestamp;
    const double lidar_t_min = lidar_scans.front().timestamp;
    const double lidar_t_max = lidar_scans.back().timestamp;
    const double overlap_begin = std::max(imu_t_min, lidar_t_min);
    const double overlap_end = std::min(imu_t_max, lidar_t_max);
    const double imu_span = imu_t_max - imu_t_min;
    const double lidar_span = lidar_t_max - lidar_t_min;
    const double overlap_span = (overlap_end > overlap_begin) ? (overlap_end - overlap_begin) : 0.0;

    UNICALIB_INFO("[Time-Align] IMU  时间范围: [{:.3f}, {:.3f}] 跨度 {:.2f}s", imu_t_min, imu_t_max, imu_span);
    UNICALIB_INFO("[Time-Align] LiDAR 时间范围: [{:.3f}, {:.3f}] 跨度 {:.2f}s", lidar_t_min, lidar_t_max, lidar_span);
    UNICALIB_INFO("[Time-Align] 重叠区间: [{:.3f}, {:.3f}] 跨度 {:.2f}s", overlap_begin, overlap_end, overlap_span);
    const double overlap_ratio = (std::min(imu_span, lidar_span) > 1e-6)
        ? (overlap_span / std::min(imu_span, lidar_span)) : 0.0;
    UNICALIB_INFO("[Time-Align] 重叠占比: {:.1f}% (相对较短流)", overlap_ratio * 100.0);

    const double imu_lidar_begin_diff = std::abs(imu_t_min - lidar_t_min);
    const double imu_lidar_end_diff = std::abs(imu_t_max - lidar_t_max);
    if (overlap_span < 10.0) {
        UNICALIB_WARN("[Time-Align] 重叠时长 < 10s，标定可能不可靠");
    }
    if (imu_lidar_begin_diff > 1.0 || imu_lidar_end_diff > 1.0) {
        UNICALIB_WARN("[Time-Align] IMU 与 LiDAR 首/末帧时间戳差异较大: 首帧差 {:.3f}s 末帧差 {:.3f}s — 若两者非同一时间基准（如不同 ROS 时钟），必须对齐后再标定",
                      imu_lidar_begin_diff, imu_lidar_end_diff);
    } else {
        UNICALIB_INFO("[Time-Align] 首/末帧时间戳差异: 首帧 {:.3f}s 末帧 {:.3f}s (同基准时通常 <1s)", imu_lidar_begin_diff, imu_lidar_end_diff);
    }
    if (imu_lidar_end_diff > 10.0)
        UNICALIB_WARN("[Time-Align] 末帧差 >10s，旋转对可能系统性偏差，手眼标定易得 180° 歧义；建议确保同一时间基准或对齐后再标定");
    UNICALIB_INFO("[Time-Align] 结论: {} (重叠>10s且首末差<1s为优)",
                  (overlap_span >= 10.0 && imu_lidar_begin_diff < 1.0 && imu_lidar_end_diff < 1.0) ? "优" : "请检查时间基准与对齐");

    try {
    // Step 1: 运行 LiDAR 里程计
    if (!run_lidar_odometry(lidar_scans)) {
        UNICALIB_ERROR("[IMU-LiDAR] LiDAR 里程计失败");
        return std::nullopt;
    }

    // ─── 在线估计陀螺零偏（基于静态段）─────────────────────────────────────────────
    auto estimate_gyro_bias_online = [](const std::vector<IMUFrame>& imu_data) -> Eigen::Vector3d {
        if (imu_data.empty()) return Eigen::Vector3d::Zero();

        std::vector<Eigen::Vector3d> static_samples;
        constexpr double k_static_gyro_thresh = 0.05;  // 与 imu_intrinsic::static_gyro_thresh 一致
        constexpr double k_static_window_max_dt = 1.0;  // 静态检测：超过此时间间隔的段跳过

        for (size_t i = 1; i < imu_data.size(); ++i) {
            double dt = imu_data[i].timestamp - imu_data[i-1].timestamp;
            if (dt > k_static_window_max_dt) continue;

            double gyro_norm = imu_data[i].gyro.norm();
            if (gyro_norm < k_static_gyro_thresh) {
                static_samples.push_back(imu_data[i].gyro);
            }
        }

        if (static_samples.empty() || static_samples.size() < 100) {
            UNICALIB_WARN("[GyroBias] 静态样本不足 ({})，使用零偏", static_samples.size());
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d bias = Eigen::Vector3d::Zero();
        for (const auto& g : static_samples) bias += g;
        bias /= static_cast<double>(static_samples.size());

        UNICALIB_INFO("[GyroBias] 估计零偏: [{:.6f}, {:.6f}, {:.6f}] rad/s (基于 {} 个静态样本)",
                      bias[0], bias[1], bias[2], static_samples.size());
        return bias;
    };

    // 估计陀螺零偏
    Eigen::Vector3d estimated_bias = Eigen::Vector3d::Zero();
    IMUIntrinsics imu_intrin_with_estimated_bias;
    if (imu_intrin) {
        imu_intrin_with_estimated_bias = *imu_intrin;
        estimated_bias = estimate_gyro_bias_online(imu_data);
        imu_intrin_with_estimated_bias.bias_gyro += estimated_bias;
    }

    // Step 2: 构建旋转对 (传入 imu_intrin_with_estimated_bias 以便积分时扣除陀螺零偏)
    auto rot_pairs = build_rotation_pairs(imu_data, &imu_intrin_with_estimated_bias);

    // 运动激励诊断（车辆等自由度受限时，仅对激励充足的参数精确标定）
    ObservabilityDiagnosis obs = compute_motion_excitation(rot_pairs);
    UNICALIB_INFO("[Motion-Excitation] 运动激励: roll={:.1f}° pitch={:.1f}° yaw={:.1f}° 轴多样性={:.3f} z平移占比={:.3f}",
                  obs.roll_motion_deg, obs.pitch_motion_deg, obs.yaw_motion_deg,
                  obs.axis_diversity, obs.z_trans_ratio);
    UNICALIB_INFO("[Motion-Excitation] 结论: {} | {}", obs.is_planar_motion ? "平面运动" : "运动较充分", obs.recommendation);
    for (const auto& w : obs.warnings)
        UNICALIB_WARN("[Motion-Excitation] {}", w);

    // Step 3: 手眼旋转标定（若有初值则用于 180° 歧义时选与初值旋转更接近的解）
    std::optional<Sophus::SO3d> handeye_prior = coarse_init.has_value() ? std::make_optional(coarse_init->so3()) : std::nullopt;
    auto rot_result = solve_handeye_rotation(rot_pairs, handeye_prior);
    if (!rot_result.has_value()) {
        UNICALIB_WARN("[IMU-LiDAR] 手眼旋转标定失败");
        return std::nullopt;
    }
    UNICALIB_INFO("  手眼旋转标定成功");
    const double handeye_residual_deg = rot_result->second;
    UNICALIB_CALC("IMU-LiDAR 手眼旋转 平均残差={:.4f} deg 旋转对数={}", handeye_residual_deg, rot_pairs.size());

    // 按激励施加先验：激励不足的 roll/pitch 置 0，仅对激励充足的自由度保留标定值
    Sophus::SO3d R_final = apply_excitation_prior_to_rotation(rot_result->first, obs);
    if (cfg_.use_planar_prior && (obs.roll_motion_deg < cfg_.min_roll_motion_deg || obs.pitch_motion_deg < cfg_.min_pitch_motion_deg)) {
        double roll_deg = std::atan2(R_final.matrix()(2, 1), R_final.matrix()(2, 2)) * (180.0 / M_PI);
        double pitch_deg = std::asin(std::max(-1.0, std::min(1.0, -R_final.matrix()(2, 0)))) * (180.0 / M_PI);
        UNICALIB_INFO("[Motion-Excitation] 已施加先验: R(RPY°)=[{:.3f}, {:.3f}, *] (激励不足分量已置0)", roll_deg, pitch_deg);
    }

    // Step 4: 估计平移（使用施加先验后的旋转）
    auto trans_result = estimate_translation(
        imu_data, R_final, 0.0);

    Eigen::Vector3d trans = trans_result.has_value() ? trans_result->first : Eigen::Vector3d::Zero();
    if (coarse_init.has_value() && trans.norm() < 0.05 && coarse_init->translation().norm() > 0.05)
        trans = coarse_init->translation();

    ExtrinsicSE3 extrinsic;
    extrinsic.ref_sensor_id = imu_id;
    extrinsic.target_sensor_id = lidar_id;
    extrinsic.SO3_TargetInRef = R_final;
    extrinsic.POS_TargetInRef = trans;
    extrinsic.time_offset_s = 0.0;
    extrinsic.handeye_rot_residual_deg = handeye_residual_deg;
    extrinsic.trans_rms_m_s = trans_result.has_value() && trans_result->second >= 0.0 ? trans_result->second : -1.0;

    UNICALIB_INFO("[IMU-LiDAR] 粗标定完成");
    const Eigen::Matrix3d R = extrinsic.SO3_TargetInRef.matrix();
    const double roll_rad = std::atan2(R(2, 1), R(2, 2));
    const double pitch_rad = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    const double yaw_rad = std::atan2(R(1, 0), R(0, 0));
    UNICALIB_INFO("[IMU-LiDAR] 粗标定摘要: R(RPY°)=[{:.3f}, {:.3f}, {:.3f}] t(m)=[{:.4f}, {:.4f}, {:.4f}] 旋转对={}",
                  roll_rad * (180.0 / M_PI), pitch_rad * (180.0 / M_PI), yaw_rad * (180.0 / M_PI),
                  extrinsic.POS_TargetInRef.x(), extrinsic.POS_TargetInRef.y(), extrinsic.POS_TargetInRef.z(),
                  rot_pairs.size());
    const double abs_roll_deg = std::abs(roll_rad * (180.0 / M_PI));
    if (abs_roll_deg > 170.0 && std::abs(pitch_rad * (180.0 / M_PI)) < 5.0 && std::abs(yaw_rad * (180.0 / M_PI)) < 5.0)
        UNICALIB_WARN("[IMU-LiDAR] 粗标定结果约 180° (RPY≈±180°,0°,0°)。若 IMU 与 LiDAR 同向安装，请检查时间对齐或设置 handeye_prefer_identity_when_ambiguous: true");
    UNICALIB_INFO("  四元数 (xyzw): {}", extrinsic.SO3_TargetInRef.unit_quaternion().coeffs().transpose());
    UNICALIB_INFO("  平移: [{:.6f}, {:.6f}, {:.6f}] m",
                  extrinsic.POS_TargetInRef.x(),
                  extrinsic.POS_TargetInRef.y(),
                  extrinsic.POS_TargetInRef.z());
    // 4x4 外参矩阵 T_Imu_Lidar (p_imu = T * p_lidar)
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = extrinsic.SO3_TargetInRef.matrix();
    T.block<3,1>(0,3) = extrinsic.POS_TargetInRef;
    UNICALIB_INFO("[IMU-LiDAR] 粗标定 4x4 T_Imu_Lidar:");
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(0,0), T(0,1), T(0,2), T(0,3));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(1,0), T(1,1), T(1,2), T(1,3));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(2,0), T(2,1), T(2,2), T(2,3));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T(3,0), T(3,1), T(3,2), T(3,3));

    return extrinsic;

    } catch (const UniCalibException& e) {
        UNICALIB_ERROR("[IMU-LiDAR] 标定异常 [{}]: {} ({}:{})",
                       errorCodeName(e.code()), e.message(), e.file(), e.line());
        return std::nullopt;
    } catch (const std::exception& e) {
        UNICALIB_ERROR("[IMU-LiDAR] 标定异常: {}", e.what());
        return std::nullopt;
    } catch (...) {
        UNICALIB_ERROR("[IMU-LiDAR] 标定未知异常");
        return std::nullopt;
    }
}

// ===================================================================
// 两阶段标定
// ===================================================================
IMULiDARCalibrator::TwoStageResult IMULiDARCalibrator::calibrate_two_stage(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin,
    const std::optional<Sophus::SE3d>& coarse_init) {

    TwoStageResult result;
    result.fine_method  = "bspline_full";

    Sophus::SE3d init_se3;
    if (coarse_init.has_value()) {
        // 粗标定以提供的初始值开始：先用手眼+平移得到粗结果，若偏离初值过远则粗结果取初值
        result.coarse_method = "handeye_from_initial";
        UNICALIB_INFO("[IMU-LiDAR] 粗标定以配置初值为先验开始 (手眼 180° 消解与平移回退)，得到粗标定结果后精标定");
        auto extrinsic = calibrate(imu_data, lidar_scans, imu_id, lidar_id, imu_intrin, coarse_init);
        if (!extrinsic.has_value()) {
            result.needs_manual = true;
            return result;
        }
        result.coarse = extrinsic;
        result.coarse_rot_err_deg = 0.0;
        init_se3 = extrinsic->SE3_TargetInRef();
        const double rot_deg = (init_se3.so3() * coarse_init->so3().inverse()).log().norm() * (180.0 / M_PI);
        const double trans_m = (init_se3.translation() - coarse_init->translation()).norm();
        if (rot_deg > 60.0 || trans_m > 0.5) {
            UNICALIB_WARN("[IMU-LiDAR] 粗标定结果偏离初值过大 (旋转 {:.1f}° 平移 {:.3f}m)，粗结果采用初值，再精标定", rot_deg, trans_m);
            init_se3 = coarse_init.value();
            result.coarse = ExtrinsicSE3{};
            result.coarse->ref_sensor_id = imu_id;
            result.coarse->target_sensor_id = lidar_id;
            result.coarse->SO3_TargetInRef = init_se3.so3();
            result.coarse->POS_TargetInRef = init_se3.translation();
            result.coarse->time_offset_s = 0.0;
            result.coarse->handeye_rot_residual_deg = -1.0;
            result.coarse->trans_rms_m_s = -1.0;
        }
    } else {
        result.coarse_method = "handeye_only";
        auto extrinsic = calibrate(imu_data, lidar_scans, imu_id, lidar_id, imu_intrin, std::nullopt);
        if (!extrinsic.has_value()) {
            result.needs_manual = true;
            return result;
        }
        result.coarse = extrinsic;
        result.coarse_rot_err_deg = 0.0;
        init_se3 = extrinsic->SE3_TargetInRef();
    }

    // 精标定：在粗标定结果上做 B 样条精细优化
    ExtrinsicSE3 refined = refine_with_spline(
        imu_data, lidar_scans, init_se3, imu_id, lidar_id, imu_intrin);
    // 将粗标定的精度指标带入精标定结果，便于写入标定文件
    if (result.coarse && result.coarse->handeye_rot_residual_deg >= 0.0)
        refined.handeye_rot_residual_deg = result.coarse->handeye_rot_residual_deg;
    if (result.coarse && result.coarse->trans_rms_m_s >= 0.0)
        refined.trans_rms_m_s = result.coarse->trans_rms_m_s;
    result.fine = refined;
    result.fine_time_offset_s = refined.time_offset_s;
    result.fine_rot_err_deg = 0.0;
    result.fine_trans_err_m  = 0.0;

    return result;
}

// ===================================================================
// LiDAR 里程计 (使用 NDT 配准)
// ===================================================================
bool IMULiDARCalibrator::run_lidar_odometry(const std::vector<LiDARScan>& scans) {
    if (scans.empty()) return false;

    UNICALIB_INFO("[LiDAR-Odom] 开始 LiDAR 里程计 (NDT 配准)");

    // 创建 LiDAROdometer
    auto lidar_odometer = LiDAROdometer::Create(
        static_cast<float>(cfg_.ndt_resolution),
        4  // 使用 4 线程进行 NDT 配准
    );

    lidar_odom_.clear();
    lidar_odom_.reserve(scans.size());

    size_t valid_frames = 0;
    for (size_t i = 0; i < scans.size(); ++i) {
        if (!scans[i].cloud || scans[i].cloud->empty()) {
            UNICALIB_WARN("[LiDAR-Odom] 帧 {} 点云为空", i);
            continue;
        }

        // 转换点云类型: pcl::PointXYZI -> IKalibrPoint
        IKalibrPointCloud::Ptr ikalibr_cloud = std::make_shared<IKalibrPointCloud>();
        ikalibr_cloud->reserve(scans[i].cloud->size());
        
        for (const auto& pt : scans[i].cloud->points) {
            // 跳过 NaN 点
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                continue;
            }
            IKalibrPoint ikalibr_pt;
            ikalibr_pt.x = pt.x;
            ikalibr_pt.y = pt.y;
            ikalibr_pt.z = pt.z;
            // 使用扫描帧的时间戳作为点云时间戳基准
            // 对于旋转式 LiDAR，如果有点级时间戳则使用，否则使用帧时间戳
            ikalibr_pt.timestamp = scans[i].timestamp;
            ikalibr_cloud->push_back(ikalibr_pt);
        }

        // 如果转换后点云为空，跳过
        if (ikalibr_cloud->empty()) {
            UNICALIB_WARN("[LiDAR-Odom] 帧 {} 转换后点云为空", i);
            continue;
        }

        // 创建 LiDARFrame
        auto frame = LiDARFrame::Create(scans[i].timestamp, ikalibr_cloud);

        // 使用 NDT 配准
        ns_ctraj::Posed pose = lidar_odometer->FeedFrame(frame);

        // 保存结果 (将 ns_ctraj::Posed 转换为 Sophus::SE3d)
        Sophus::SO3d so3(pose.so3.matrix());
        Sophus::SE3d se3_pose(so3, pose.t);
        lidar_odom_.emplace_back(scans[i].timestamp, se3_pose);

        valid_frames++;

        if (progress_cb_) {
            progress_cb_("LiDAR-Odom", static_cast<double>(i + 1) / scans.size());
        }
    }

    double pts_min = 1e9, pts_max = 0, pts_sum = 0;
    for (const auto& s : scans) {
        if (s.cloud && !s.cloud->empty()) {
            size_t n = s.cloud->size();
            if (n < pts_min) pts_min = n;
            if (n > pts_max) pts_max = n;
            pts_sum += n;
        }
    }
    UNICALIB_INFO("[LiDAR-Odom] 完成, 处理 {} 帧 (有效 {} 帧)", scans.size(), valid_frames);
    if (valid_frames > 0)
        UNICALIB_INFO("[LiDAR-Odom] 点云规模: 每帧点数 min={} med≈{} max={}",
                      pts_min, static_cast<size_t>(pts_sum / valid_frames), pts_max);
    return valid_frames > 0;
}

// ===================================================================
// 构建旋转对 (简化实现)
// ===================================================================
std::vector<LiDARRotPair> IMULiDARCalibrator::build_rotation_pairs(
    const std::vector<IMUFrame>& imu_data,
    const IMUIntrinsics* imu_intrin) {

    std::vector<LiDARRotPair> pairs;
    if (imu_data.empty() || lidar_odom_.size() < 2) return pairs;

    UNICALIB_INFO("[Build-RotPairs] 构建旋转对");
    UNICALIB_INFO("[Build-RotPairs] 计算逻辑: 同一时间段 [t_begin,t_end] 内 — LiDAR 相对旋转 = (T_w_l0^{-1}*T_w_l1).so3().inverse() 即机体从 t_begin→t_end 的旋转；IMU 相对旋转 = 陀螺积分 R(t_begin→t_end)。二者同为机体旋转，应一致");
    UNICALIB_INFO("[Build-RotPairs] 时间基准要求: t_begin/t_end 使用 LiDAR 帧时间戳，IMU 积分区间与之一致；IMU 与 LiDAR 必须为同一时间基准（如 ROS bag 时间），未对齐会导致旋转对错误、标定偏差甚至约 180° 误差");

    // 为每对相邻的 LiDAR 里程计位姿， 创建旋转对
    for (size_t i = 1; i < lidar_odom_.size(); ++i) {
        const auto& pose0 = lidar_odom_[i - 1];
        const auto& pose1 = lidar_odom_[i];

        double t_begin = pose0.first;
        double t_end = pose1.first;

        // 计算两帧之间的相对位姿 (T_lidar0_lidar1 = T_w_l0^{-1} * T_w_l1)，其旋转部分 R_l0^T*R_l1 表示「从 t1 到 t0」的坐标变换；
        // 手眼要求与 IMU 一致的是「从 t_begin 到 t_end」的机体旋转 R_body = R_w_l1^T*R_w_l0 = (rel_pose.so3())^{-1}，故取逆
        Sophus::SE3d rel_pose = pose0.second.inverse() * pose1.second;
        Sophus::SO3d rot_lidar = rel_pose.so3().inverse();
        double rel_trans_norm = rel_pose.translation().norm();

        // 从 IMU 数据中积分得到对应的旋转 (传入 imu_intrin 时扣除陀螺零偏)
        IMULiDARCalibrator::IntegrationStats stats;
        Sophus::SO3d rot_imu = integrate_imu_rotation(imu_data, t_begin, t_end, imu_intrin, (i == 1) ? &stats : nullptr);
        if (i == 1 && stats.num_samples > 0) {
            UNICALIB_INFO("[Build-RotPairs] 首对积分统计: dt={:.3f}s 采样数={} 陀螺1σ漂移≈{:.4f}° (ARW)",
                          stats.dt_s, stats.num_samples, stats.sigma_deg);
            UNICALIB_INFO("[Build-RotPairs] 首对时间对齐: LiDAR 区间 [t_begin,t_end]=[{:.4f}, {:.4f}] | IMU 参与积分区间 [{:.4f}, {:.4f}]",
                          t_begin, t_end, stats.t_first_imu, stats.t_last_imu);
            const double actual_span = stats.t_last_imu - stats.t_first_imu;
            UNICALIB_INFO("[Build-RotPairs] 首对 IMU 积分计算: sum_dt={:.6f}s 实际时间跨度={:.6f}s 请求区间={:.6f}s (sum_dt 应≈实际跨度)",
                          stats.sum_dt, actual_span, stats.dt_s);
            UNICALIB_INFO("[Build-RotPairs] 首对 IMU 积分: 步长直接使用话题时间戳 dt=t_i-t_{i-1} → dt_min={:.6f}s dt_median={:.6f}s dt_max={:.6f}s 跳过步数={} (200Hz 标称 0.005s)",
                          stats.dt_min, stats.dt_median, stats.dt_max, stats.num_skipped);
            if (stats.dt_min < 0.001 && stats.dt_min > 0)
                UNICALIB_WARN("[Build-RotPairs] 存在过小时间戳间隔 dt_min={:.6f}s（可能重复戳），已跳过 <0.1ms 的步；建议检查 bag 或 IMU 驱动", stats.dt_min);
            const double gap_begin = std::abs(stats.t_first_imu - t_begin);
            const double gap_end = std::abs(stats.t_last_imu - t_end);
            if (gap_begin > 0.02 || gap_end > 0.02) {
                UNICALIB_WARN("[Build-RotPairs] IMU 与 LiDAR 区间边界差异: 首端 {:.3f}s 末端 {:.3f}s — 若时间基准不一致，必须对齐后再标定", gap_begin, gap_end);
            }
            if (gap_begin > 1e-6 || gap_end > 1e-6) {
                UNICALIB_INFO("[Build-RotPairs] 首对区间裁剪: 积分未覆盖 t_begin~t_first 约 {:.4f}s, t_last~t_end 约 {:.4f}s (可能导致 IMU 角略小于真实)",
                              gap_begin, gap_end);
            }
            if (stats.dt_s > 1.0)
                UNICALIB_WARN("[Build-RotPairs] 积分时长>1s 漂移显著，建议启用短时窗 rot_pair_dt_target_s<=0.5 或增加 LiDAR 帧数");
        }
        // 旋转角度(弧度) = log(R).norm()，与配置中的 min_motion_rot_deg 比较
        if (rot_imu.log().norm() > cfg_.min_motion_rot_deg * MathSafety::DEG_TO_RAD) {
            LiDARRotPair pair;
            pair.t_begin = t_begin;
            pair.t_end = t_end;
            pair.rot_LiDAR = rot_lidar;
            pair.rot_IMU = rot_imu;
            pair.rel_trans_norm = rel_trans_norm;
            
            // 质量评估：基于平移量和旋转一致性
            // - 平移过小（<0.1m）时 NDT 可能退化，降权
            // - LiDAR/IMU 旋转比值偏离 1 较多时，降权
            double rot_imu_deg = MathSafety::radToDeg(rot_imu.log().norm());
            double rot_lidar_deg = MathSafety::radToDeg(rot_lidar.log().norm());
            double ratio = (rot_imu_deg > 0.5) ? (rot_lidar_deg / rot_imu_deg) : 1.0;
            
            pair.quality = 1.0;
            if (rel_trans_norm < 0.1) {
                // 平移过小，NDT 可能退化
                pair.quality *= 0.5;
            }
            if (ratio < 0.5 || ratio > 2.0) {
                // 旋转比值严重偏离，质量降权
                pair.quality *= 0.3;
            } else if (ratio < 0.7 || ratio > 1.5) {
                // 旋转比值中度偏离
                pair.quality *= 0.7;
            }
            
            pairs.push_back(pair);
        }
    }

    const size_t n_consecutive = pairs.size();
    const double t_min = lidar_odom_.front().first;
    const double t_max = lidar_odom_.back().first;
    const double frame_dt_med = (lidar_odom_.size() > 1)
        ? (t_max - t_min) / static_cast<double>(lidar_odom_.size() - 1) : 0;

    // 短时窗旋转对：用插值位姿 + 短时陀螺积分，降低长时积分漂移（研究结论：误差∝sqrt(dt)）
    // 放宽触发条件：只要帧间隔 > dt_target 即启用（原 frame_dt_med > 1.0 导致 0.5s 间隔时短时窗未启用）
    if (cfg_.rot_pair_dt_target_s > 0 && lidar_odom_.size() >= 2 &&
        (frame_dt_med > cfg_.rot_pair_dt_target_s || n_consecutive == 0)) {
        auto interp_pose = [this](double t) -> Sophus::SE3d {
            if (lidar_odom_.empty()) return Sophus::SE3d();
            if (t <= lidar_odom_.front().first) return lidar_odom_.front().second;
            if (t >= lidar_odom_.back().first) return lidar_odom_.back().second;
            size_t i = 0;
            while (i + 1 < lidar_odom_.size() && lidar_odom_[i + 1].first < t) ++i;
            double t0 = lidar_odom_[i].first, t1 = lidar_odom_[i + 1].first;
            double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
            alpha = std::max(0.0, std::min(1.0, alpha));
            const Sophus::SE3d& T0 = lidar_odom_[i].second, & T1 = lidar_odom_[i + 1].second;
            Sophus::SO3d R = Sophus::SO3d::exp(alpha * (T1.so3() * T0.so3().inverse()).log()) * T0.so3();
            Eigen::Vector3d p = (1.0 - alpha) * T0.translation() + alpha * T1.translation();
            return Sophus::SE3d(R, p);
        };
        const double dt_target = cfg_.rot_pair_dt_target_s;
        const double step = std::max(dt_target * 0.5, cfg_.rot_pair_short_step_s);
        int added_short = 0;
        IntegrationStats short_stats;
        for (double t = t_min; t + dt_target <= t_max; t += step) {
            Sophus::SE3d P0 = interp_pose(t);
            Sophus::SE3d P1 = interp_pose(t + dt_target);
            Sophus::SO3d rot_lidar = (P0.inverse() * P1).so3().inverse();  // 与相邻帧一致：取「t→t+dt 机体旋转」
            Sophus::SO3d rot_imu = integrate_imu_rotation(imu_data, t, t + dt_target, imu_intrin,
                                                          (added_short == 0) ? &short_stats : nullptr);
            double rot_imu_deg = MathSafety::radToDeg(rot_imu.log().norm());
            if (rot_imu_deg < cfg_.min_motion_rot_deg_short) continue;
            LiDARRotPair pair;
            pair.t_begin = t;
            pair.t_end = t + dt_target;
            pair.rot_LiDAR = rot_lidar;
            pair.rot_IMU = rot_imu;
            pair.quality = 0.9;
            pair.rel_trans_norm = (P1.translation() - P0.translation()).norm();
            pairs.push_back(pair);
            added_short++;
        }
        if (added_short > 0) {
            UNICALIB_INFO("[Build-RotPairs] 短时窗旋转对: dt_target={:.2f}s 步长={:.2f}s 新增 {} 对 (降低陀螺积分漂移)",
                          dt_target, step, added_short);
            UNICALIB_INFO("[Build-RotPairs] 短时窗首对积分: dt={:.3f}s 采样数={} 1σ漂移≈{:.4f}°",
                          short_stats.dt_s, short_stats.num_samples, short_stats.sigma_deg);
        }
    }

    UNICALIB_INFO("[Build-RotPairs] 构建了 {} 个有效旋转对 (相邻帧 {} + 短时窗 {}, 最小激励 {:.1f} deg)",
                  pairs.size(), n_consecutive, pairs.size() - n_consecutive, cfg_.min_motion_rot_deg);
    if (!pairs.empty()) {
        double min_imu = 1e9, max_imu = 0, min_ld = 1e9, max_ld = 0;
        std::vector<double> dt_spans, imu_degs, ld_degs, trans_norms;
        trans_norms.reserve(pairs.size());
        for (const auto& p : pairs) {
            double d_imu = MathSafety::radToDeg(p.rot_IMU.log().norm());
            double d_ld = MathSafety::radToDeg(p.rot_LiDAR.log().norm());
            double dt_span = p.t_end - p.t_begin;
            imu_degs.push_back(d_imu);
            ld_degs.push_back(d_ld);
            dt_spans.push_back(dt_span);
            if (d_imu < min_imu) min_imu = d_imu;
            if (d_imu > max_imu) max_imu = d_imu;
            if (d_ld < min_ld) min_ld = d_ld;
            if (d_ld > max_ld) max_ld = d_ld;
        }
        // 每对 LiDAR 相对平移 (m)：若旋转和平移都偏小，说明 NDT 可能退化或收敛到近恒等
        for (const auto& p : pairs) {
            trans_norms.push_back(p.rel_trans_norm);
        }
        UNICALIB_INFO("[Build-RotPairs] IMU 相对旋转: {:.2f} ~ {:.2f} deg | LiDAR 相对旋转: {:.2f} ~ {:.2f} deg",
                      min_imu, max_imu, min_ld, max_ld);
        if (!trans_norms.empty()) {
            std::vector<double> tn = trans_norms;
            std::sort(tn.begin(), tn.end());
            UNICALIB_INFO("[Build-RotPairs] LiDAR 相对平移(相邻帧): min={:.3f} med={:.3f} max={:.3f} m (平移过小且旋转过小则 NDT 可能退化)",
                          tn.front(), tn[tn.size()/2], tn.back());
        }
        std::sort(dt_spans.begin(), dt_spans.end());
        UNICALIB_INFO("[Build-RotPairs] 旋转对时间跨度 dt_span(s): min={:.3f} med={:.3f} max={:.3f} (大跨度会放大里程计漂移与时间不同步)",
                      dt_spans.front(), dt_spans[dt_spans.size()/2], dt_spans.back());
        // LiDAR vs IMU 角度比：理想应接近 1（同一次运动）
        double sum_ratio = 0;
        int count_ratio = 0;
        std::vector<double> ratios;
        ratios.reserve(pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (imu_degs[i] > 0.5) {
                double r = ld_degs[i] / imu_degs[i];
                sum_ratio += r;
                ratios.push_back(r);
                count_ratio++;
            }
        }
        double ratio_mean = count_ratio > 0 ? (sum_ratio / count_ratio) : 0.0;
        double ratio_std = 0.0;
        if (count_ratio > 1) {
            for (double r : ratios) ratio_std += (r - ratio_mean) * (r - ratio_mean);
            ratio_std = std::sqrt(ratio_std / (count_ratio - 1));
        }
        int n_lo = 0, n_mid = 0, n_ok = 0, n_hi = 0;
        for (double r : ratios) {
            if (r < 0.5) n_lo++;
            else if (r < 0.8) n_mid++;
            else if (r <= 1.2) n_ok++;
            else n_hi++;
        }
        if (count_ratio > 0) {
            std::sort(ratios.begin(), ratios.end());
            double ratio_med = ratios[ratios.size() / 2];
            double ratio_p25 = ratios.size() >= 4 ? ratios[ratios.size() / 4] : ratio_med;
            double ratio_p75 = ratios.size() >= 4 ? ratios[ratios.size() * 3 / 4] : ratio_med;
            UNICALIB_INFO("[Build-RotPairs] LiDAR角/IMU角 比值: 均值={:.3f} 中位数={:.3f} 标准差={:.3f} (理想≈1)", ratio_mean, ratio_med, ratio_std);
            UNICALIB_INFO("[Build-RotPairs] 比值分位: P25={:.3f} P50={:.3f} P75={:.3f}", ratio_p25, ratio_med, ratio_p75);
            UNICALIB_INFO("[Build-RotPairs] 比值分布: ratio<0.5 共 {} 对, 0.5~0.8 共 {} 对, 0.8~1.2 共 {} 对, >1.2 共 {} 对",
                          n_lo, n_mid, n_ok, n_hi);
            const char* quality = (ratio_mean >= 0.7 && n_ok + n_hi >= static_cast<int>(pairs.size()) / 2) ? "优" :
                (ratio_mean >= 0.4 && n_lo < static_cast<int>(pairs.size()) / 2) ? "中" : "差";
            UNICALIB_INFO("[Build-RotPairs] 旋转对质量: {} (优=比值近1且多数在0.8~1.2)", quality);
            if (ratio_mean > 0 && (ratio_mean < 0.7 || ratio_mean > 1.3))
                UNICALIB_WARN("[Build-RotPairs] 比值均值 {:.3f} 系统性偏离 1，手眼易出现 180° 歧义；请检查时间对齐/帧间隔，或配置 handeye_prefer_identity_when_ambiguous: true", ratio_mean);
        }
        if (ratio_mean > 0 && ratio_mean < 0.3) {
            UNICALIB_WARN("[Build-RotPairs] LiDAR 相对旋转远小于 IMU，可能原因: (1) NDT 在大时间间隔(dt≈{:.1f}s)下未正确估计旋转 (2) 场景平面/退化 (3) 建议增大 data.lidar.max_frames 使帧间隔<1s 或换用更密 LiDAR 帧",
                          dt_spans.empty() ? 0.0 : dt_spans[dt_spans.size()/2]);
        }
        // 按比值过滤低质量旋转对（可选）：比值偏离 1 过多说明该对不可靠
        if ((cfg_.handeye_ratio_min > 0 || cfg_.handeye_ratio_max > 0) && !pairs.empty()) {
            const double rmin = cfg_.handeye_ratio_min > 0 ? cfg_.handeye_ratio_min : 0.0;
            const double rmax = cfg_.handeye_ratio_max > 0 ? cfg_.handeye_ratio_max : 1e9;
            std::vector<LiDARRotPair> filtered;
            filtered.reserve(pairs.size());
            for (size_t i = 0; i < pairs.size(); ++i) {
                double r = (imu_degs[i] > 0.5) ? (ld_degs[i] / imu_degs[i]) : -1.0;
                if (r < 0 || (r >= rmin && r <= rmax))
                    filtered.push_back(pairs[i]);
            }
            if (filtered.size() < pairs.size()) {
                UNICALIB_INFO("[Build-RotPairs] 按比值 [{:.2f}, {:.2f}] 过滤: {} -> {} 对", rmin, rmax, pairs.size(), filtered.size());
                pairs = std::move(filtered);
            }
        }
        // 抽样打印前 5 对与后 5 对: pair#, dt_span, rot_imu_deg, rot_lidar_deg, ratio, trans_m
        auto log_sample = [&](const std::vector<size_t>& indices) {
            for (size_t idx : indices) {
                if (idx >= pairs.size()) continue;
                const auto& p = pairs[idx];
                double dt = p.t_end - p.t_begin;
                double di = MathSafety::radToDeg(p.rot_IMU.log().norm());
                double dl = MathSafety::radToDeg(p.rot_LiDAR.log().norm());
                double r = (di > 0.5) ? (dl / di) : 0.0;
                UNICALIB_INFO("[Build-RotPairs]   pair#{} dt={:.2f}s rot_imu={:.2f}° rot_lidar={:.2f}° ratio={:.3f} trans={:.3f}m",
                              idx, dt, di, dl, r, p.rel_trans_norm);
            }
        };
        UNICALIB_INFO("[Build-RotPairs] 抽样 (前5对):");
        log_sample({0, 1, 2, 3, 4});
        if (pairs.size() > 10) {
            UNICALIB_INFO("[Build-RotPairs] 抽样 (后5对):");
            log_sample({pairs.size()-5, pairs.size()-4, pairs.size()-3, pairs.size()-2, pairs.size()-1});
        }
    }
    return pairs;
}

// ===================================================================
// IMU 旋转积分 (改进实现) + 可选统计（用于漂移预估与日志）
// 漂移: 角随机游走(ARW)导致 1σ 误差 ≈ noise_gyro * sqrt(dt) [rad]，长时积分偏差大
// ===================================================================
Sophus::SO3d IMULiDARCalibrator::integrate_imu_rotation(
    const std::vector<IMUFrame>& imu_data,
    double t_begin,
    double t_end,
    const IMUIntrinsics* imu_intrin,
    IntegrationStats* out_stats) {
    // 找到时间范围内的 IMU 数据
    std::vector<IMUFrame> segment;
    for (const auto& frame : imu_data) {
        if (frame.timestamp >= t_begin && frame.timestamp <= t_end) {
            segment.push_back(frame);
        }
    }

    // 过小 dt 视为重复/近重复时间戳，不参与积分（仅统计为跳过）
    constexpr double k_dt_dup_threshold_s = 0.0001;  // 0.1ms，小于此视为重复时间戳

    if (out_stats) {
        out_stats->dt_s = t_end - t_begin;
        out_stats->num_samples = static_cast<int>(segment.size());
        out_stats->sigma_deg = 0;
        out_stats->sum_dt = 0;
        out_stats->dt_min = 0;
        out_stats->dt_median = 0;
        out_stats->dt_max = 0;
        out_stats->num_skipped = 0;
        if (!segment.empty()) {
            out_stats->t_first_imu = segment.front().timestamp;
            out_stats->t_last_imu = segment.back().timestamp;
        } else {
            out_stats->t_first_imu = out_stats->t_last_imu = 0.0;
        }
        if (imu_intrin && out_stats->dt_s > 0) {
            // 角随机游走: σ_θ(rad) = N_arw * sqrt(dt), noise_gyro 通常即 N_arw [rad/s/√Hz]
            double sigma_rad = imu_intrin->noise_gyro * std::sqrt(out_stats->dt_s);
            out_stats->sigma_deg = MathSafety::radToDeg(sigma_rad);
        }
    }

    if (segment.empty()) {
        UNICALIB_WARN("[IMU-Integrate] 时间范围内无 IMU 数据");
        return Sophus::SO3d();
    }

    if (segment.size() < 2) {
        UNICALIB_WARN("[IMU-Integrate] IMU 数据点太少");
        return Sophus::SO3d();
    }

    // 按 IMU 消息时间戳排序（dt 直接由话题时间戳计算：dt = t_i - t_{i-1}）
    std::sort(segment.begin(), segment.end(),
              [](const IMUFrame& a, const IMUFrame& b) {
                  return a.timestamp < b.timestamp;
              });

    // 中值积分：步长 dt 直接使用相邻两帧 IMU 话题时间戳之差
    Sophus::SO3d rot = Sophus::SO3d();
    double sum_dt = 0;
    double dt_min = 1e9;
    double dt_max = 0;
    int num_skipped = 0;
    std::vector<double> dts_for_median;  // 用于计算 dt 中位数（仅有效步）

    for (size_t i = 1; i < segment.size(); ++i) {
        // 直接使用 IMU 话题时间戳计算本步间隔
        double dt = segment[i].timestamp - segment[i - 1].timestamp;

        if (dt <= 0 || dt > 1.0) {
            if (out_stats) out_stats->num_skipped++;
            num_skipped++;
            UNICALIB_WARN("[IMU-Integrate] 异常时间间隔: dt={:.6f}s 步 [{},{}]", dt, i - 1, i);
            continue;
        }
        if (dt < k_dt_dup_threshold_s) {
            if (out_stats) out_stats->num_skipped++;
            num_skipped++;
            continue;  // 视为重复/近重复时间戳，不积分
        }
        sum_dt += dt;
        if (dt < dt_min) dt_min = dt;
        if (dt > dt_max) dt_max = dt;
        dts_for_median.push_back(dt);

        Eigen::Vector3d avg_gyro = 0.5 * (segment[i - 1].gyro + segment[i].gyro);
        if (imu_intrin) {
            avg_gyro -= imu_intrin->bias_gyro;
        }
        if (!avg_gyro.allFinite()) {
            UNICALIB_WARN("[IMU-Integrate] 角速度包含无效值");
            continue;
        }
        double gyro_norm = avg_gyro.norm();
        if (gyro_norm > 10.0) {
            UNICALIB_WARN("[IMU-Integrate] 角速度过大: {:.4f} rad/s, 可能是噪声", gyro_norm);
            continue;
        }

        Eigen::Vector3d delta_rot = avg_gyro * dt;
        rot = rot * Sophus::SO3d::exp(delta_rot);
    }

    if (out_stats) {
        out_stats->sum_dt = sum_dt;
        out_stats->dt_min = (dt_min <= 1e8) ? dt_min : 0.0;
        out_stats->dt_max = dt_max;
        out_stats->num_skipped = num_skipped;
        if (!dts_for_median.empty()) {
            std::sort(dts_for_median.begin(), dts_for_median.end());
            out_stats->dt_median = dts_for_median[dts_for_median.size() / 2];
        }
    }

    return rot;
}

// ===================================================================
// 手眼旋转标定 (使用 SVD 分解方法)
// ===================================================================
std::optional<std::pair<Sophus::SO3d, double>> IMULiDARCalibrator::solve_handeye_rotation(
    const std::vector<LiDARRotPair>& pairs,
    const std::optional<Sophus::SO3d>& handeye_prior) {
    if (pairs.empty()) {
        UNICALIB_ERROR("[Handeye] 旋转对为空");
        return std::nullopt;
    }

    UNICALIB_INFO("[IMU-LiDAR] 步骤: 手眼旋转标定开始 — 旋转对 {} 个", pairs.size());
    UNICALIB_INFO("[Handeye] 方程: R_imu*X = X*R_lidar，X=R_LidarInImu。歧义时: decision={} margin_deg={:.1f}°",
                  cfg_.handeye_180_decision, cfg_.handeye_180_residual_margin_deg);

    std::vector<LiDARRotPair> work = pairs;
    const int max_outlier_iter = std::max(1, cfg_.handeye_outlier_max_iter);
    const double reject_quantile = std::max(0.0, std::min(1.0, cfg_.handeye_outlier_reject_quantile));
    const int min_pairs = std::max(10, cfg_.handeye_outlier_min_pairs);

    // RANSAC 手眼：最小集 2 对求解，取内点最多的一组再做加权 SVD，抑制离群对与 180° 歧义
    if (cfg_.handeye_ransac_enable && pairs.size() >= 3u) {
        const double ransac_thresh_deg = std::max(0.5, cfg_.handeye_ransac_inlier_thresh_deg);
        const int ransac_max_iter = std::max(10, cfg_.handeye_ransac_max_iter);
        const size_t Np = pairs.size();
        int best_inlier_count = 0;
        std::vector<size_t> best_inlier_indices;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dist(0, Np - 1);

        for (int iter = 0; iter < ransac_max_iter; ++iter) {
            size_t i = dist(gen);
            size_t j = dist(gen);
            if (i == j) { j = (j + 1) % Np; if (i == j) continue; }
            Eigen::MatrixXd M = buildHandeyeM(pairs, {i, j}, false);
            auto X_opt = solveHandeyeFromM(M);
            if (!X_opt) continue;
            std::vector<double> err_deg = handeyeResidualsDeg(pairs, *X_opt);
            int inlier_count = 0;
            std::vector<size_t> inlier_idx;
            for (size_t k = 0; k < Np; ++k) {
                if (err_deg[k] < ransac_thresh_deg) {
                    inlier_count++;
                    inlier_idx.push_back(k);
                }
            }
            if (inlier_count > best_inlier_count) {
                best_inlier_count = inlier_count;
                best_inlier_indices = std::move(inlier_idx);
            }
        }

        if (best_inlier_count >= static_cast<int>(min_pairs) && !best_inlier_indices.empty()) {
            work.clear();
            work.reserve(best_inlier_indices.size());
            for (size_t k : best_inlier_indices) work.push_back(pairs[k]);
            UNICALIB_INFO("[Handeye] RANSAC 内点数 {} (阈值 {:.1f}°)，使用内点做加权 SVD",
                          best_inlier_count, ransac_thresh_deg);
        } else {
            UNICALIB_INFO("[Handeye] RANSAC 内点不足 (最佳 {} < {})，回退到全量 SVD",
                          best_inlier_count, min_pairs);
        }
    }

    Eigen::Vector4d q_vec;
    std::vector<double> err_deg_vec;
    double avg_error_deg = 0.0;
    double min_err_deg = 1e9;
    double max_err_deg = 0.0;
    double std_err = 0.0;

        for (int iter = 0; iter < max_outlier_iter; ++iter) {
        const size_t N = work.size();
        // ─── 构建 SVD 约束矩阵（带质量权重）───────────────────────────────────────
        // 使用 pair.quality 作为权重，质量越高（接近1.0）权重越大
        std::vector<double> weights;
        weights.reserve(N);
        double sum_weight = 0.0;
        for (size_t i = 0; i < N; ++i) {
            weights.push_back(work[i].quality);
            sum_weight += weights[i];
        }
        UNICALIB_INFO("[Handeye] 质量权重: min={:.3f} max={:.3f} avg={:.3f}",
                      *std::min_element(weights.begin(), weights.end()),
                      *std::max_element(weights.begin(), weights.end()),
                      sum_weight / N);

        Eigen::MatrixXd M(4 * static_cast<int>(N), 4);
        M.setZero();
        for (size_t i = 0; i < N; ++i) {
            const auto& R_A = work[i].rot_IMU;   // IMU 旋转
            const auto& R_B = work[i].rot_LiDAR;  // LiDAR 旋转
            double weight = weights[i];  // 质量权重

            Eigen::Quaterniond q_A = R_A.unit_quaternion();
            Eigen::Quaterniond q_B = R_B.unit_quaternion();

            // 确保四元数符号一致性
            if (q_A.w() < 0) q_A.coeffs() = -q_A.coeffs();
            if (q_B.w() < 0) q_B.coeffs() = -q_B.coeffs();

            // 构建约束矩阵（每个约束乘以权重 sqrt(weight)，实现加权最小二乘）
            double sqrt_w = std::sqrt(std::max(weight, 0.001));  // 避免零权重
            double row[4][4];

            // (q_A.w - q_B.w) * q_X.w - (q_A.vec + q_B.vec) · q_X.vec = 0
            row[0][0] = q_A.w() - q_B.w();
            row[0][1] = -q_A.x() - q_B.x();
            row[0][2] = -q_A.y() - q_B.y();
            row[0][3] = -q_A.z() - q_B.z();

            // (q_A.x + q_B.x) * q_X.w + (q_A.w + q_B.w) * q_X.x + (q_A.z - q_B.z) * q_X.y - (q_A.y + q_B.y) * q_X.z = 0
            row[1][0] = q_A.x() + q_B.x();
            row[1][1] = q_A.w() + q_B.w();
            row[1][2] = q_A.z() - q_B.z();
            row[1][3] = -q_A.y() - q_B.y();

            // (q_A.y + q_B.y) * q_X.w + (q_A.y + q_B.z) * q_X.x + (q_A.w + q_B.w) * q_X.y - (q_A.z + q_B.x) * q_X.z = 0
            row[2][0] = q_A.y() + q_B.y();
            row[2][1] = q_A.y() - q_B.z();
            row[2][2] = q_A.w() + q_B.w();
            row[2][3] = -q_A.z() - q_B.x();

            // (q_A.z + q_B.z) * q_X.w + (q_A.z + q_B.y) * q_X.x + (q_A.y + q_B.x) * q_X.y + (q_A.w + q_B.w) * q_X.z = 0
            row[3][0] = q_A.z() + q_B.z();
            row[3][1] = q_A.z() + q_B.y();
            row[3][2] = q_A.y() + q_B.x();
            row[3][3] = q_A.w() + q_B.w();

            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    M(4 * i + r, c) = row[r][c] * sqrt_w;
                }
            }
        }
    
    // 使用 SVD 求解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    Eigen::Vector4d sv = svd.singularValues();
    const double sv_ratio = (sv(0) > 1e-12) ? (sv(3) / sv(0)) : 0.0;
    if (iter == 0) {
        UNICALIB_INFO("[Handeye] 约束矩阵 M ({}x4) 奇异值: {:.4f} {:.4f} {:.4f} {:.4f} (最小过小表示病态)",
                      M.rows(), sv(0), sv(1), sv(2), sv(3));
        UNICALIB_INFO("[Handeye] 奇异值比 sv_min/sv_max = {:.4f} (越接近0越病态，>0.05 较健康)", sv_ratio);
    }
    q_vec = svd.matrixV().col(3);  // 最小奇异值对应的向量
    
    // 归一化四元数
    double q_norm = q_vec.norm();
    if (q_norm < 1e-10) {
        UNICALIB_ERROR("[Handeye] SVD 结果接近零，无法求解有效四元数");
        return std::nullopt;
    }
    
    q_vec /= q_norm;
    
    // 确保四元数实部为正（符号一致性）
    if (q_vec(3) < 0) {
        q_vec = -q_vec;
    }
    
    // 构建四元数 (Eigen::Quaterniond 存储顺序是 [x, y, z, w])
    Eigen::Quaterniond q(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    
    // 计算残差用于诊断
    double total_error = 0.0;
    max_err_deg = 0.0;
    min_err_deg = 1e9;
    err_deg_vec.clear();
    err_deg_vec.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        const auto& R_A = work[i].rot_IMU;
        const auto& R_B = work[i].rot_LiDAR;
        Sophus::SO3d X(q);
        Sophus::SO3d lhs = R_A * X;
        Sophus::SO3d rhs = X * R_B;
        Sophus::SO3d err = lhs * rhs.inverse();
        double e_deg = MathSafety::radToDeg(err.log().norm());
        total_error += err.log().norm();
        err_deg_vec.push_back(e_deg);
        if (e_deg > max_err_deg) max_err_deg = e_deg;
        if (e_deg < min_err_deg) min_err_deg = e_deg;
    }
    avg_error_deg = MathSafety::radToDeg(total_error / static_cast<double>(N));
    std_err = 0.0;
    if (N > 1) {
        for (double e : err_deg_vec) std_err += (e - avg_error_deg) * (e - avg_error_deg);
        std_err = std::sqrt(std_err / (N - 1));
    }
    // 离群点剔除：按残差分位数剔除高残差对并重解
    if (reject_quantile > 0 && iter < max_outlier_iter - 1) {
        std::vector<double> es = err_deg_vec;
        std::sort(es.begin(), es.end());
        const size_t idx = std::min(static_cast<size_t>(es.size() * reject_quantile), es.size() - 1);
        const double thresh = es.empty() ? 0 : es[idx];
        std::vector<LiDARRotPair> next_work;
        for (size_t i = 0; i < work.size(); ++i)
            if (err_deg_vec[i] <= thresh) next_work.push_back(work[i]);
        const int removed = static_cast<int>(work.size() - next_work.size());
        if (removed == 0 || next_work.size() < static_cast<size_t>(min_pairs))
            break;
        UNICALIB_INFO("[Handeye] 离群点剔除 迭代{}: 移除 {} 对 (残差>{}°)，剩余 {} 对", iter + 1, removed, thresh, next_work.size());
        work = std::move(next_work);
    } else {
        break;
    }
    }  // end for(iter)

    const size_t N = work.size();
    std::vector<double> err_sorted = err_deg_vec;
    std::sort(err_sorted.begin(), err_sorted.end());
    double err_p50 = err_sorted.empty() ? 0 : err_sorted[err_sorted.size() / 2];
    double err_p90 = err_sorted.size() >= 10 ? err_sorted[err_sorted.size() * 9 / 10] : (err_sorted.empty() ? 0 : err_sorted.back());
    UNICALIB_INFO("[IMU-LiDAR] 步骤: 手眼旋转标定完成 — 平均误差: {:.4f} deg (min={:.4f} max={:.4f} std={:.4f})",
                  avg_error_deg, min_err_deg, max_err_deg, std_err);
    UNICALIB_INFO("[Handeye] 残差分位: P50={:.3f}° P90={:.3f}°", err_p50, err_p90);
    if (avg_error_deg > 10.0) {
        UNICALIB_WARN("[Handeye] 平均误差较大，建议检查数据质量（LiDAR 里程计/IMU 时间对齐/运动激励）");
    }
    // 残差最大的 3 对：便于定位是哪些时间段的运动导致不一致
    {
        std::vector<size_t> idx(work.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::partial_sort(idx.begin(), idx.begin() + std::min<size_t>(3, idx.size()), idx.end(),
                          [&err_deg_vec](size_t a, size_t b) { return err_deg_vec[a] > err_deg_vec[b]; });
        UNICALIB_INFO("[Handeye] 残差最大 3 对 (用于定位异常时段):");
        for (size_t k = 0; k < std::min<size_t>(3, work.size()); ++k) {
            size_t i = idx[k];
            double dt_span = work[i].t_end - work[i].t_begin;
            UNICALIB_INFO("  pair#{} t=[{:.1f},{:.1f}]s dt_span={:.2f}s 残差={:.2f} deg",
                          i, work[i].t_begin, work[i].t_end, dt_span, err_deg_vec[i]);
        }
    }
    // 手眼旋转结果 3x3 矩阵 (外参旋转部分)
    Eigen::Quaterniond q_final(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    Sophus::SO3d X_final(q_final);
    Eigen::Matrix3d R_handeye = X_final.matrix();
    double trace_R = R_handeye.trace();
    const double rot_angle_deg = MathSafety::radToDeg(std::acos(std::max(-1.0, std::min(1.0, (trace_R - 1.0) * 0.5))));
    UNICALIB_INFO("[Handeye] SVD 初解: trace(R)={:.4f} 相对单位阵旋转角={:.3f}° (0°=单位阵 180°=歧义解)", trace_R, rot_angle_deg);
    bool used_180_fix = false;
    double avg_alt_deg = -1.0;  // 单位阵侧解平均残差 [deg]，仅当 trace_R<-0.5 时有效
    if (cfg_.handeye_fix_180_ambiguity && trace_R < -0.5) {
        const Sophus::SO3d R_180_x = Sophus::SO3d::exp(M_PI * Eigen::Vector3d::UnitX());
        Sophus::SO3d X_alt = X_final * R_180_x;
        double total_alt = 0.0;
        for (size_t i = 0; i < N; ++i) {
            Sophus::SO3d lhs = work[i].rot_IMU * X_alt;
            Sophus::SO3d rhs = X_alt * work[i].rot_LiDAR;
            total_alt += (lhs * rhs.inverse()).log().norm();
        }
        avg_alt_deg = MathSafety::radToDeg(total_alt / static_cast<double>(N));
        UNICALIB_INFO("[Handeye] 180° 歧义两解: 180°侧 残差={:.4f}° | 单位阵侧 残差={:.4f}° (策略: {})",
                      avg_error_deg, avg_alt_deg, cfg_.handeye_180_decision);
        const double margin_deg = std::max(0.0, cfg_.handeye_180_residual_margin_deg);
        bool use_identity_side = false;
        if (handeye_prior.has_value()) {
            const double dist_180_deg = MathSafety::radToDeg((X_final * handeye_prior->inverse()).log().norm());
            const double dist_id_deg = MathSafety::radToDeg((X_alt * handeye_prior->inverse()).log().norm());
            use_identity_side = (dist_id_deg < dist_180_deg);
            UNICALIB_INFO("[Handeye] 初值先验: 180°侧距初值 {:.2f}° | 单位阵侧距初值 {:.2f}° → 采用{}", dist_180_deg, dist_id_deg, use_identity_side ? "单位阵侧" : "180°侧");
        } else if (cfg_.handeye_180_decision == "residual") {
            // 纯按残差：谁小选谁，适合外参可能真是 180° 的安装
            use_identity_side = (avg_alt_deg < avg_error_deg);
            UNICALIB_INFO("[Handeye] 策略=residual: {} 残差更小，采用{}",
                          use_identity_side ? "单位阵侧" : "180°侧", use_identity_side ? "单位阵侧解" : "180°解");
        } else if (cfg_.handeye_180_decision == "prefer_180") {
            // 反向安装先验：优先 180°，仅当单位阵侧残差优出 margin_deg 才选单位阵
            const bool identity_better_enough = (avg_alt_deg + margin_deg < avg_error_deg);
            use_identity_side = identity_better_enough;
            if (use_identity_side)
                UNICALIB_INFO("[Handeye] 采用单位阵侧解 (prefer_180 下单位阵侧优出 {:.2f}° >= margin {:.1f}°)",
                              avg_error_deg - avg_alt_deg, margin_deg);
            else
                UNICALIB_INFO("[Handeye] 保留 180° 解 (策略=prefer_180，单位阵未优出 {:.1f}°)", margin_deg);
        } else {
            // prefer_identity（默认）：同向安装先验，仅当 180° 解残差明显更优才保留 180°
            const bool prefer_identity = cfg_.handeye_prefer_identity_when_ambiguous;
            const bool keep_180_because_better = (avg_error_deg + margin_deg < avg_alt_deg);
            use_identity_side = (prefer_identity && !keep_180_because_better) || (avg_alt_deg < avg_error_deg);
            if (use_identity_side) {
                if (prefer_identity && !keep_180_because_better)
                    UNICALIB_INFO("[Handeye] 采用单位阵侧解 (prefer_identity，且 180° 未优出 {:.1f}°)", margin_deg);
                else
                    UNICALIB_INFO("[Handeye] 单位阵侧残差更小 ({:.4f}° < {:.4f}°)，采用单位阵侧解", avg_alt_deg, avg_error_deg);
            } else {
                if (prefer_identity)
                    UNICALIB_WARN("[Handeye] 保留 180° 解：180° 侧残差比单位阵侧优 {:.2f}° > margin {:.1f}°。若 IMU/LiDAR 同向安装，请检查时间对齐或设 handeye_180_decision: residual 后按残差选",
                                  avg_alt_deg - avg_error_deg, margin_deg);
                else
                    UNICALIB_INFO("[Handeye] 保留 180° 解 (当前残差 {:.4f}° <= 单位阵侧 {:.4f}°)", avg_error_deg, avg_alt_deg);
            }
        }
        if (use_identity_side) {
            X_final = X_alt;
            R_handeye = X_final.matrix();
            used_180_fix = true;
        }
    }
    const double final_rot_angle_deg =
        MathSafety::radToDeg(std::acos(std::max(-1.0, std::min(1.0, (R_handeye.trace() - 1.0) * 0.5))));
    UNICALIB_INFO("[Handeye] 最终解: {} (相对单位阵旋转角={:.3f}°)",
                  used_180_fix ? "单位阵侧(已校正)" : (trace_R < -0.5 ? "180°侧" : "近单位阵"),
                  final_rot_angle_deg);
    if (trace_R < -0.5 && !used_180_fix) {
        UNICALIB_WARN("[Handeye] 最终采用 180° 解。若 IMU 与 LiDAR 同向安装，请: (1) 检查 IMU/LiDAR 时间基准与对齐 (2) 设置 handeye_prefer_identity_when_ambiguous: true (3) 检查旋转对 LiDAR角/IMU角 比值是否系统性偏离 1");
    }
    UNICALIB_INFO("[Handeye] 旋转矩阵 R_Imu_Lidar (3x3):");
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}]", R_handeye(0,0), R_handeye(0,1), R_handeye(0,2));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}]", R_handeye(1,0), R_handeye(1,1), R_handeye(1,2));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}]", R_handeye(2,0), R_handeye(2,1), R_handeye(2,2));
    const double final_mean_residual_deg = (cfg_.handeye_fix_180_ambiguity && trace_R < -0.5)
        ? (used_180_fix ? avg_alt_deg : avg_error_deg) : avg_error_deg;
    return std::make_pair(X_final, final_mean_residual_deg);
}

// ===================================================================
// 平移估计 (基于 LiDAR 里程计速度 + IMU 角速度约束)
// 原理: omega x t = v_imu - R * v_lidar
// 使用 MAD 滤除离群点， 保持简洁
// ===================================================================
std::optional<std::pair<Eigen::Vector3d, double>> IMULiDARCalibrator::estimate_translation(
    const std::vector<IMUFrame>& imu_data,
    const Sophus::SO3d& rot_LiDAR_in_IMU,
    double time_offset) {

    if (lidar_odom_.size() < 2) {
        UNICALIB_ERROR("[Trans-Est] LiDAR 里程计数据不足");
        return std::nullopt;
    }

    UNICALIB_INFO("[IMU-LiDAR] 步骤: 平移估计开始 (里程计 {} 帧)", lidar_odom_.size());
    // 帧间 dt 统计：若 LiDAR 为采样后的非连续帧，dt 会很大导致全部被过滤
    {
        std::vector<double> dts;
        for (size_t i = 1; i < lidar_odom_.size(); ++i)
            dts.push_back(lidar_odom_[i].first - lidar_odom_[i - 1].first);
        if (!dts.empty()) {
            std::sort(dts.begin(), dts.end());
            double dt_min = dts.front(), dt_max = dts.back();
            double dt_med = dts[dts.size() / 2];
            double t_span = lidar_odom_.back().first - lidar_odom_.front().first;
            UNICALIB_INFO("[Trans-Est] 帧间 dt(s): min={:.3f} med={:.3f} max={:.3f} (有效范围 [0.05, 2.0]); 总时间跨度 {:.1f}s",
                          dt_min, dt_med, dt_max, t_span);
            if (dt_min > 2.0 || dt_max > 2.0)
                UNICALIB_WARN("[Trans-Est] 所有 dt>2.0s，因 LiDAR 为采样非连续帧；建议: 提高 data.lidar.max_frames 或放宽平移估计 dt 上限");
        }
    }
    // 阈值参数
    constexpr double MIN_VELOCITY = 0.1;       // 最小速度 [m/s]
    constexpr double MIN_GYRO = 0.05;          // 最小角速度 [rad/s]
    constexpr double MAX_VELOCITY = 10.0;      // 最大速度 [m/s]
    constexpr double MAX_GYRO = 5.0;           // 最大角速度 [rad/s]
    constexpr size_t MIN_CONSTRAINTS = 8;      // 最小约束数
    constexpr double MAD_THRESHOLD = 2.5;      // MAD 离群点阈值

    Eigen::Matrix3d R = rot_LiDAR_in_IMU.matrix();

    // 收集约束数据
    struct Constraint {
        Eigen::Vector3d omega;     // IMU 角速度
        Eigen::Vector3d vel_diff;  // v_imu - R * v_lidar
        double residual_norm;      // 用于离群点检测
    };
    std::vector<Constraint> constraints;
    size_t skip_dt = 0, skip_vel = 0, skip_gyro = 0, skip_imu_count = 0;

    for (size_t i = 1; i < lidar_odom_.size(); ++i) {
        const auto& [t0, pose0] = lidar_odom_[i - 1];
        const auto& [t1, pose1] = lidar_odom_[i];
        double dt = t1 - t0;
        if (dt < 0.05 || dt > 2.0) { skip_dt++; continue; }

        // LiDAR 速度 (LiDAR 坐标系)
        Eigen::Vector3d v_lidar = (pose1.translation() - pose0.translation()) / dt;
        double v_norm = v_lidar.norm();
        if (v_norm < MIN_VELOCITY || v_norm > MAX_VELOCITY) { skip_vel++; continue; }

        // IMU 平均角速度
        Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();
        int count = 0;
        for (const auto& frame : imu_data) {
            if (frame.timestamp >= t0 && frame.timestamp <= t1) {
                avg_gyro += frame.gyro;
                count++;
            }
        }
        if (count == 0) { skip_imu_count++; continue; }
        avg_gyro /= count;

        double gyro_norm = avg_gyro.norm();
        if (gyro_norm < MIN_GYRO || gyro_norm > MAX_GYRO) { skip_gyro++; continue; }

        // 约束: omega x t = v_imu - R * v_lidar
        // 计算 v_imu: 通过 LiDAR 里程计差分 + IMU 加速度积分
        // v_imu 可以从以下方式估计:
        // 方法1: LiDAR里程计速度差分 (推荐，精度高)
        // 方法2: 如果数据充足， IMU预积分 (备选, 更鲁棒但需要IMU内参)
        // 方法3: 如果无里程计信息, 使用零速度假设 (向后兼容)
        
        Eigen::Vector3d v_imu = Eigen::Vector3d::Zero();
        
        // 方法1: 从 LiDAR 里程计估计 v_imu (使用相邻帧差分)
        if (i >= 2) {
            // 计算里程计帧间的速度 (在 LiDAR 坐标系下)
            const Sophus::SE3d& T_w_lidar_curr = pose1;
            const Sophus::SE3d& T_w_lidar_prev = pose0;
            
            // LiDAR 在世界坐标系的速度
            Eigen::Vector3d v_lidar_w = (T_w_lidar_curr.translation() - T_w_lidar_prev.translation()) / dt;
            
            // 变换到 IMU 坐标系
            Eigen::Vector3d v_lidar_imu = rot_LiDAR_in_IMU * v_lidar_w;
            v_imu = v_lidar_imu;
        } else {
            // 方法2: IMU 预积分 (如果提供了 IMU 内参和有足够的数据)
            // 注意: 这需要准确的 IMU 内参, 且数据充足
            // 当前简化实现使用里程计差分
            
            // 方法3: 鰧零速度假设 (向后兼容)
            // 当无法从里程计或 IMU 获取有效速度估计时使用零速度
            v_imu.setZero();
            UNICALIB_DEBUG("[Trans-Est] 无法估计 v_imu, 使用零速度假设 (可能降低精度)");
        }
        
        Constraint c;
        c.omega = avg_gyro;
        c.vel_diff = v_imu - R * v_lidar;
        c.residual_norm = 0.0;      // 后续计算
        constraints.push_back(c);
    }

    // 若相邻帧约束不足（例如 LiDAR 为采样非连续帧导致 dt 全超 2s），则按时间戳插值生成约束
    if (constraints.size() < MIN_CONSTRAINTS) {
        UNICALIB_INFO("[Trans-Est] 相邻帧约束不足 ({} 个)，尝试按 IMU/LiDAR 时间戳插值生成约束", constraints.size());
        UNICALIB_INFO("[Trans-Est] 诊断: 跳过 dt 无效 {} 对, 速度 {} 对, 角速度 {} 对, 无 IMU 采样 {} 对 (总相邻帧 {} 对)",
                      skip_dt, skip_vel, skip_gyro, skip_imu_count, lidar_odom_.size() > 0 ? lidar_odom_.size() - 1 : 0);

        const double t_min = lidar_odom_.front().first;
        const double t_max = lidar_odom_.back().first;
        auto interp_pose = [&](double t) -> Sophus::SE3d {
            if (lidar_odom_.empty()) return Sophus::SE3d();
            if (t <= lidar_odom_.front().first) return lidar_odom_.front().second;
            if (t >= lidar_odom_.back().first) return lidar_odom_.back().second;
            size_t i = 0;
            while (i + 1 < lidar_odom_.size() && lidar_odom_[i + 1].first < t) ++i;
            double t0 = lidar_odom_[i].first, t1 = lidar_odom_[i + 1].first;
            double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
            alpha = std::max(0.0, std::min(1.0, alpha));
            const Sophus::SE3d& T0 = lidar_odom_[i].second, & T1 = lidar_odom_[i + 1].second;
            Sophus::SO3d R = Sophus::SO3d::exp(alpha * (T1.so3() * T0.so3().inverse()).log()) * T0.so3();
            Eigen::Vector3d p = (1.0 - alpha) * T0.translation() + alpha * T1.translation();
            return Sophus::SE3d(R, p);
        };

        constexpr double DT_TARGET = 0.1;       // 按 0.1s 间隔生成约束，与 IMU 时间尺度匹配
        constexpr double T_STEP = 0.05;        // 时间步长 50ms，可重叠采样
        size_t added = 0;
        for (double t = t_min; t + DT_TARGET <= t_max; t += T_STEP) {
            double dt = DT_TARGET;
            Sophus::SE3d pose0 = interp_pose(t);
            Sophus::SE3d pose1 = interp_pose(t + dt);
            Eigen::Vector3d v_lidar = (pose1.translation() - pose0.translation()) / dt;
            double v_norm = v_lidar.norm();
            if (v_norm < MIN_VELOCITY || v_norm > MAX_VELOCITY) continue;

            Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();
            int count = 0;
            for (const auto& frame : imu_data) {
                if (frame.timestamp >= t && frame.timestamp <= t + dt) {
                    avg_gyro += frame.gyro;
                    count++;
                }
            }
            if (count == 0) continue;
            avg_gyro /= count;
            double gyro_norm = avg_gyro.norm();
            if (gyro_norm < MIN_GYRO || gyro_norm > MAX_GYRO) continue;

            Eigen::Vector3d v_lidar_w = (pose1.translation() - pose0.translation()) / dt;
            Eigen::Vector3d v_imu = rot_LiDAR_in_IMU * v_lidar_w;
            Constraint c;
            c.omega = avg_gyro;
            c.vel_diff = v_imu - R * v_lidar;
            c.residual_norm = 0.0;
            constraints.push_back(c);
            added++;
        }
        UNICALIB_INFO("[Trans-Est] 按时间戳插值: 在 [t_min={:.1f}, t_max={:.1f}] 内 dt={:.2f}s 步长={:.2f}s 新增 {} 个约束",
                      t_min, t_max, DT_TARGET, T_STEP, added);
    }

    if (constraints.size() < MIN_CONSTRAINTS) {
        UNICALIB_WARN("[Trans-Est] 有效约束仍不足: {} < {} (需平移+角速度激励或更多数据)", constraints.size(), MIN_CONSTRAINTS);
        return std::make_pair(Eigen::Vector3d::Zero(), -1.0);
    }

    UNICALIB_INFO("[Trans-Est] 共收集 {} 个约束", constraints.size());

    // === 使用 MAD 滤除离群点 ===
    // 先用所有数据做初步估计
    auto solve_constraints = [](const std::vector<Constraint>& cons) -> Eigen::Vector3d {
        const size_t N = cons.size();
        Eigen::MatrixXd A(3 * N, 3);
        Eigen::VectorXd b(3 * N);
        for (size_t i = 0; i < N; ++i) {
            const auto& w = cons[i].omega;
            A.row(3*i+0) << 0, -w.z(), w.y();
            A.row(3*i+1) << w.z(), 0, -w.x();
            A.row(3*i+2) << -w.y(), w.x(), 0;
            b.segment<3>(3*i) = cons[i].vel_diff;
        }
        return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    };

    Eigen::Vector3d t_init = solve_constraints(constraints);

    // 计算残差
    std::vector<double> residuals;
    for (auto& c : constraints) {
        Eigen::Matrix3d skew;
        skew << 0, -c.omega.z(), c.omega.y(),
                c.omega.z(), 0, -c.omega.x(),
                -c.omega.y(), c.omega.x(), 0;
        Eigen::Vector3d pred = skew * t_init;
        c.residual_norm = (pred - c.vel_diff).norm();
        residuals.push_back(c.residual_norm);
    }

    // 计算 MAD (Median Absolute Deviation)
    std::vector<double> sorted_residuals = residuals;
    std::sort(sorted_residuals.begin(), sorted_residuals.end());
    double median = sorted_residuals[sorted_residuals.size() / 2];
    std::vector<double> abs_dev;
    for (double r : residuals) abs_dev.push_back(std::abs(r - median));
    std::sort(abs_dev.begin(), abs_dev.end());
    double mad = abs_dev[abs_dev.size() / 2] * 1.4826;  // 缩放因子

    // 滤除离群点
    std::vector<Constraint> filtered;
    for (const auto& c : constraints) {
        double z_score = (mad > 1e-6) ? std::abs(c.residual_norm - median) / mad : 0.0;
        if (z_score < MAD_THRESHOLD) {
            filtered.push_back(c);
        }
    }

    UNICALIB_INFO("[Trans-Est] MAD 滤波: {} -> {} 约束", constraints.size(), filtered.size());

    if (filtered.size() < MIN_CONSTRAINTS) {
        UNICALIB_WARN("[Trans-Est] 滤波后约束不足， 返回初步估计");
        UNICALIB_INFO("  t = [{:.3f}, {:.3f}, {:.3f}] m", t_init.x(), t_init.y(), t_init.z());
        return std::make_pair(t_init, -1.0);
    }

    // 最终求解
    Eigen::Vector3d t_final = solve_constraints(filtered);

    // 计算最终残差
    double rms_error = 0.0;
    for (const auto& c : filtered) {
        Eigen::Matrix3d skew;
        skew << 0, -c.omega.z(), c.omega.y(),
                c.omega.z(), 0, -c.omega.x(),
                -c.omega.y(), c.omega.x(), 0;
        Eigen::Vector3d pred = skew * t_final;
        double err = (pred - c.vel_diff).norm();
        rms_error += err * err;
    }
    rms_error = std::sqrt(rms_error / filtered.size());

    // 平面运动时平移 z 不可观，可选固定为 0
    if (cfg_.use_planar_prior && lidar_odom_.size() >= 2) {
        double sum_norm = 0, sum_z_abs = 0;
        const Sophus::SO3d R0 = lidar_odom_.front().second.so3();
        for (size_t i = 1; i < lidar_odom_.size(); ++i) {
            Eigen::Vector3d dt = lidar_odom_[i].second.translation() - lidar_odom_[i - 1].second.translation();
            sum_norm += dt.norm();
            sum_z_abs += std::abs((R0.inverse() * dt).z());
        }
        double z_ratio = (sum_norm > 1e-6) ? (sum_z_abs / sum_norm) : 0.0;
        if (z_ratio <= cfg_.max_z_trans_ratio) {
            t_final.z() = 0.0;
            UNICALIB_INFO("[Trans-Est] 平面运动 (z平移占比={:.3f})：平移 z 固定为 0，仅标定 x/y", z_ratio);
        }
    }

    UNICALIB_CALC("IMU-LiDAR 平移估计 t=[{:.4f},{:.4f},{:.4f}] m rms_m_s={:.4f} 约束数={}",
                  t_final.x(), t_final.y(), t_final.z(), rms_error, filtered.size());
    UNICALIB_INFO("[IMU-LiDAR] 步骤: 平移估计完成 — t=[{:.3f}, {:.3f}, {:.3f}] m, RMS={:.4f} m/s",
                  t_final.x(), t_final.y(), t_final.z(), rms_error);
    UNICALIB_INFO("  t = [{:.3f}, {:.3f}, {:.3f}] m", t_final.x(), t_final.y(), t_final.z());
    UNICALIB_INFO("  RMS 误差 = {:.4f} m/s (基于 {} 个约束)", rms_error, filtered.size());

    if (rms_error > 0.3) {
        UNICALIB_WARN("[Trans-Est] 残差较大 ({:.3f} m/s)， 结果可能不准确", rms_error);
    }

    return std::make_pair(t_final, rms_error);
}

// ===================================================================
// 辅助: 在 (timestamp, SE3) 序列上线性/Slerp 插值
// ===================================================================
namespace {
Sophus::SE3d interpolate_pose(
    const std::vector<std::pair<double, Sophus::SE3d>>& poses,
    double t) {
    if (poses.empty()) return Sophus::SE3d();
    if (poses.size() == 1) return poses[0].second;
    if (t <= poses.front().first) return poses.front().second;
    if (t >= poses.back().first) return poses.back().second;

    size_t i = 0;
    while (i + 1 < poses.size() && poses[i + 1].first < t) ++i;
    double t0 = poses[i].first, t1 = poses[i + 1].first;
    double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
    alpha = std::max(0.0, std::min(1.0, alpha));

    const Sophus::SE3d& T0 = poses[i].second;
    const Sophus::SE3d& T1 = poses[i + 1].second;
    Sophus::SO3d R = Sophus::SO3d::exp(alpha * (T1.so3() * T0.so3().inverse()).log()) * T0.so3();
    Eigen::Vector3d p = (1.0 - alpha) * T0.translation() + alpha * T1.translation();
    return Sophus::SE3d(R, p);
}

// Ceres 代价: 外参(6) + 时间偏移(1) vs 固定 B 样条与 LiDAR 位姿
// 使用 basalt-headers 的 pose() 与 d_pose_d_t() 提供精确（解析）雅可比，替代数值微分
struct SplineExtrinsicCost {
    const basalt::Se3Spline<4>* spline;
    int64_t t_lidar_ns;
    Sophus::SE3d T_w_lidar;

    bool operator()(const double* const extr_rot_trans, const double* const time_offset,
                    double* residual) const {
        Eigen::Map<const Eigen::Vector3d> rot_vec(extr_rot_trans);
        Eigen::Map<const Eigen::Vector3d> trans(extr_rot_trans + 3);
        double dt = time_offset[0];

        int64_t t_query_ns = t_lidar_ns + static_cast<int64_t>(1e9 * dt);
        Sophus::SE3d T_w_imu = spline->pose(t_query_ns);
        Sophus::SE3d T_imu_lidar(Sophus::SO3d::exp(rot_vec), trans);
        Sophus::SE3d T_w_lidar_pred = T_w_imu * T_imu_lidar;
        Eigen::Matrix<double, 6, 1> log_err =
            (T_w_lidar_pred.inverse() * T_w_lidar).log();
        for (int i = 0; i < 6; ++i) residual[i] = log_err(i);
        return true;
    }
};

// 解析雅可比代价函数：基于 basalt-headers 的 d_pose_d_t 实现精确估计
// 残差对 外参(6) 与 时间偏移(1) 的导数由解析公式给出，避免数值微分误差
class SplineExtrinsicAnalyticCost : public ceres::CostFunction {
 public:
    SplineExtrinsicAnalyticCost(const basalt::Se3Spline<4>* spline,
                                int64_t t_lidar_ns,
                                const Sophus::SE3d& T_w_lidar)
        : spline_(spline), t_lidar_ns_(t_lidar_ns), T_w_lidar_(T_w_lidar) {
        set_num_residuals(6);
        mutable_parameter_block_sizes()->push_back(6);   // extr
        mutable_parameter_block_sizes()->push_back(1);   // time_offset
    }

    bool Evaluate(double const* const* parameters,
                  double* residuals,
                  double** jacobians) const override {
        const double* extr = parameters[0];
        const double dt = parameters[1][0];
        Eigen::Map<const Eigen::Vector3d> rot_vec(extr);
        Eigen::Map<const Eigen::Vector3d> trans(extr + 3);

        const int64_t t_query_ns = t_lidar_ns_ + static_cast<int64_t>(1e9 * dt);
        Sophus::SE3d T_w_imu = spline_->pose(t_query_ns);
        Sophus::SE3d T_imu_lidar(Sophus::SO3d::exp(rot_vec), trans);
        Sophus::SE3d T_w_lidar_pred = T_w_imu * T_imu_lidar;
        Sophus::SE3d T_err = T_w_lidar_pred.inverse() * T_w_lidar_;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
        r = T_err.log();

        if (!jacobians) return true;

        // 使用 basalt 的 decoupled SE(3) 右雅可比逆: r = log(T_err), dr/d(eps_pred) = -J_r^{-1}(r)
        Eigen::Matrix<double, 6, 6> J_r_inv;
        Sophus::rightJacobianInvSE3Decoupled(r, J_r_inv);
        // 外参 extr = [rot_vec(3), trans(3)]，T_imu_lidar = (exp(rot_vec), trans)，decoupled tangent phi = (trans, rot_vec)
        // d(phi)/d(extr) = [0 I; I 0]，且 tangent_pred (body) = Ad(T_imu_lidar^{-1}) * phi_imu_lidar
        Eigen::Matrix<double, 6, 6> d_phi_imu_lidar_d_extr;
        d_phi_imu_lidar_d_extr.setZero();
        d_phi_imu_lidar_d_extr.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        d_phi_imu_lidar_d_extr.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> Ad_inv = T_imu_lidar.inverse().Adj();
        Eigen::Matrix<double, 6, 6> d_res_d_extr =
            -J_r_inv * Ad_inv * d_phi_imu_lidar_d_extr;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J_extr(jacobians[0]);
            J_extr = d_res_d_extr;
        }

        // d(residual)/d(time_offset): d(residual)/d(dt) = d(residual)/d(T_pred) * d(T_pred)/d(t) * 1e9
        // basalt: d_pose_d_t 给出 body 系角速度与线速度 (omega, v_body)
        if (jacobians[1]) {
            Eigen::Matrix<double, 6, 1> d_pose_d_t;
            spline_->d_pose_d_t(t_query_ns, d_pose_d_t);
            // T_pred 对 t 的导数 (在 T_w_imu 处): d(T_w_imu)/dt = T_w_imu * [omega_body; v_body] (右扰动)
            // 在 T_pred 的 body 系: d(T_pred)/d(t) = T_imu_lidar^{-1}.Adj() * d_pose_d_t
            Eigen::Matrix<double, 6, 1> tangent_pred = T_imu_lidar.inverse().Adj() * d_pose_d_t;
            Eigen::Matrix<double, 6, 1> d_res_d_t = -J_r_inv * tangent_pred * 1e9;
            jacobians[1][0] = d_res_d_t(0);
            jacobians[1][1] = d_res_d_t(1);
            jacobians[1][2] = d_res_d_t(2);
            jacobians[1][3] = d_res_d_t(3);
            jacobians[1][4] = d_res_d_t(4);
            jacobians[1][5] = d_res_d_t(5);
        }
        return true;
    }

 private:
    const basalt::Se3Spline<4>* spline_;
    int64_t t_lidar_ns_;
    Sophus::SE3d T_w_lidar_;
};
}  // namespace

// ===================================================================
// B样条精细优化 (完整实现)
// 流程: LiDAR 里程计 → T_w_imu 序列 → SE3 B样条 → 优化外参 + 时间偏移
// ===================================================================
ExtrinsicSE3 IMULiDARCalibrator::refine_with_spline(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const Sophus::SE3d& init_extrinsic,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin) {

    UNICALIB_INFO("[IMU-LiDAR] 步骤: B样条精细优化开始 (LiDAR {} 帧)", lidar_scans.size());
    ExtrinsicSE3 result;
    result.ref_sensor_id = imu_id;
    result.target_sensor_id = lidar_id;
    result.SO3_TargetInRef = init_extrinsic.so3();
    result.POS_TargetInRef = init_extrinsic.translation();
    result.time_offset_s = 0.0;

    if (lidar_scans.empty()) {
        UNICALIB_WARN("[B-spline] LiDAR 数据为空，返回初值");
        return result;
    }

    // Step 1: 运行 LiDAR 里程计
    if (!run_lidar_odometry(lidar_scans)) {
        UNICALIB_WARN("[B-spline] LiDAR 里程计失败，返回初值");
        return result;
    }
    if (lidar_odom_.size() < 4) {
        UNICALIB_WARN("[B-spline] 里程计帧数不足 (need >= 4)，返回初值");
        return result;
    }

    // T_lidar_imu = T_imu_lidar^{-1}，即 LiDAR 在 IMU 系下位姿的逆
    Sophus::SE3d T_lidar_imu = init_extrinsic.inverse();
    std::vector<std::pair<double, Sophus::SE3d>> T_w_imu_poses;
    T_w_imu_poses.reserve(lidar_odom_.size());
    for (const auto& [t, T_w_lidar] : lidar_odom_)
        T_w_imu_poses.emplace_back(t, T_w_lidar * T_lidar_imu);

    double t_min = T_w_imu_poses.front().first;
    double t_max = T_w_imu_poses.back().first;
    double dt_s = std::max(0.05, cfg_.spline_dt_s);
    const int64_t dt_ns = static_cast<int64_t>(dt_s * 1e9);
    const int64_t t0_ns = static_cast<int64_t>(t_min * 1e9);

    basalt::Se3Spline<4> spline(static_cast<int64_t>(dt_s * 1e9), t0_ns);
    int num_knots = static_cast<int>(std::ceil((t_max - t_min) / dt_s)) + 4;
    num_knots = std::max(num_knots, 8);

    for (int i = 0; i < num_knots; ++i) {
        double t = t_min + i * dt_s;
        spline.knotsPushBack(interpolate_pose(T_w_imu_poses, t));
    }

    // Step 2: Ceres 优化外参 (6) + 时间偏移 (1)
    double extr[6];
    Eigen::Vector3d rvec = result.SO3_TargetInRef.log();
    extr[0] = rvec(0); extr[1] = rvec(1); extr[2] = rvec(2);
    extr[3] = result.POS_TargetInRef(0);
    extr[4] = result.POS_TargetInRef(1);
    extr[5] = result.POS_TargetInRef(2);
    double time_offset_s = result.time_offset_s;

    ceres::Problem problem;
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

    for (size_t i = 0; i < lidar_odom_.size(); ++i) {
        int64_t t_ns = static_cast<int64_t>(lidar_odom_[i].first * 1e9);
        if (t_ns < spline.minTimeNs() || t_ns > spline.maxTimeNs())
            continue;
        ceres::CostFunction* cost =
            new SplineExtrinsicAnalyticCost(&spline, t_ns, lidar_odom_[i].second);
        problem.AddResidualBlock(cost, loss, extr, &time_offset_s);
    }

    problem.SetParameterLowerBound(&time_offset_s, 0, -cfg_.time_offset_max_s);
    problem.SetParameterUpperBound(&time_offset_s, 0, cfg_.time_offset_max_s);

    ceres::Solver::Options options;
    options.max_num_iterations = cfg_.ceres_max_iter;
    options.minimizer_progress_to_stdout = cfg_.verbose;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    UNICALIB_CALC("IMU-LiDAR B样条 Ceres initial_cost={:.6f} final_cost={:.6f} iter={} time_offset_s={:.4f} converged={}",
                  summary.initial_cost, summary.final_cost, static_cast<int>(summary.iterations.size()),
                  time_offset_s, summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");
    if (cfg_.verbose)
        UNICALIB_INFO("[B-spline] Ceres: {} iterations, cost={:.6f}",
                      summary.iterations.size(), summary.final_cost);

    result.SO3_TargetInRef = Sophus::SO3d::exp(Eigen::Vector3d(extr[0], extr[1], extr[2]));
    result.POS_TargetInRef = Eigen::Vector3d(extr[3], extr[4], extr[5]);
    result.time_offset_s = time_offset_s;
    result.is_converged = (summary.termination_type == ceres::CONVERGENCE);
    result.bspline_final_cost = summary.final_cost;

    UNICALIB_INFO("[IMU-LiDAR] 步骤: B样条精细优化完成 — time_offset={:.4f}s converged={} cost={:.6f} iter={}",
                  result.time_offset_s, result.is_converged ? "yes" : "no",
                  summary.final_cost, summary.iterations.size());
    UNICALIB_INFO("[B-spline] 代价: 初={:.6f} 终={:.6f} (迭代{} 次)",
                  summary.initial_cost, summary.final_cost, summary.iterations.size());
    // 初值 vs 精化: 旋转差(deg)、平移差(m)
    Sophus::SO3d R_init = init_extrinsic.so3();
    double rot_diff_deg = MathSafety::radToDeg((result.SO3_TargetInRef * R_init.inverse()).log().norm());
    double trans_diff = (result.POS_TargetInRef - init_extrinsic.translation()).norm();
    UNICALIB_INFO("[B-spline] 初值→精化: 旋转差 {:.4f} deg, 平移差 {:.6f} m", rot_diff_deg, trans_diff);
    // 精标定结果 4x4 矩阵
    Eigen::Matrix4d T_fine = Eigen::Matrix4d::Identity();
    T_fine.block<3,3>(0,0) = result.SO3_TargetInRef.matrix();
    T_fine.block<3,1>(0,3) = result.POS_TargetInRef;
    UNICALIB_INFO("[B-spline] 精标定 4x4 T_Imu_Lidar:");
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T_fine(0,0), T_fine(0,1), T_fine(0,2), T_fine(0,3));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T_fine(1,0), T_fine(1,1), T_fine(1,2), T_fine(1,3));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T_fine(2,0), T_fine(2,1), T_fine(2,2), T_fine(2,3));
    UNICALIB_INFO("  [{:.6f}, {:.6f}, {:.6f}, {:.6f}]", T_fine(3,0), T_fine(3,1), T_fine(3,2), T_fine(3,3));

    return result;
}

// ===================================================================
// 手动校准验证
// ===================================================================
ExtrinsicSE3 IMULiDARCalibrator::calibrate_manual_verify(
    const std::vector<IMUFrame>& /*imu_data*/,
    const std::vector<LiDARScan>& /*lidar_scans*/,
    const ExtrinsicSE3& init_extrin) {
    // 最小实现：返回初值并打日志。完整实现需可视化旋转对比 + 增量调整或单步 B 样条 refine。
    UNICALIB_INFO("[IMU-LiDAR] calibrate_manual_verify: 使用初值（完整手动验证 TODO）");
    ExtrinsicSE3 out = init_extrin;
    out.is_converged = false;  // 未做验证步骤，不标记为已收敛
    return out;
}

// ===================================================================
// 运动激励诊断：车辆等运动自由度受限时，识别 roll/pitch/yaw 与平移各向激励是否充足
// ===================================================================
IMULiDARCalibrator::ObservabilityDiagnosis IMULiDARCalibrator::compute_motion_excitation(
    const std::vector<LiDARRotPair>& pairs) const {

    ObservabilityDiagnosis obs;
    if (pairs.empty()) {
        obs.warnings.push_back("旋转对为空，无法统计运动激励");
        return obs;
    }

    // 各轴旋转量（弧度）：对每对相对旋转的 log 向量在 body X/Y/Z 上投影的绝对值求和
    double sum_roll_rad = 0, sum_pitch_rad = 0, sum_yaw_rad = 0;
    Eigen::Matrix3d axis_cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pairs) {
        Eigen::Vector3d log_vec = p.rot_IMU.log();
        double angle = log_vec.norm();
        if (angle < 1e-9) continue;
        Eigen::Vector3d axis = log_vec / angle;
        sum_roll_rad += std::abs(log_vec.x());
        sum_pitch_rad += std::abs(log_vec.y());
        sum_yaw_rad += std::abs(log_vec.z());
        axis_cov += axis * axis.transpose();
    }
    obs.roll_motion_deg = MathSafety::radToDeg(sum_roll_rad);
    obs.pitch_motion_deg = MathSafety::radToDeg(sum_pitch_rad);
    obs.yaw_motion_deg = MathSafety::radToDeg(sum_yaw_rad);

    const double total_rot_deg = obs.roll_motion_deg + obs.pitch_motion_deg + obs.yaw_motion_deg;
    if (total_rot_deg > 1e-6) {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(axis_cov, Eigen::ComputeFullU);
        Eigen::Vector3d sv = svd.singularValues();
        double ssum = sv.sum();
        obs.axis_diversity = (ssum > 1e-12) ? (1.0 - sv(0) / ssum) : 0.0;  // 0=单轴 1=各向均匀
    }

    // Z 平移占比：从里程计相邻帧位移在首帧体系的 Z 分量占比
    if (lidar_odom_.size() >= 2) {
        double sum_trans_norm = 0, sum_trans_z_abs = 0;
        const Sophus::SO3d R0 = lidar_odom_.front().second.so3();
        for (size_t i = 1; i < lidar_odom_.size(); ++i) {
            Eigen::Vector3d dt = lidar_odom_[i].second.translation() - lidar_odom_[i - 1].second.translation();
            Eigen::Vector3d dt_in_first = R0.inverse() * dt;
            sum_trans_norm += dt.norm();
            sum_trans_z_abs += std::abs(dt_in_first.z());
        }
        obs.z_trans_ratio = (sum_trans_norm > 1e-6) ? (sum_trans_z_abs / sum_trans_norm) : 0.0;
    }
    obs.is_planar_motion = (obs.z_trans_ratio <= cfg_.max_z_trans_ratio);

    if (cfg_.enable_planar_warning) {
        if (obs.roll_motion_deg < cfg_.min_roll_motion_deg)
            obs.warnings.push_back("roll 激励不足 (" + std::to_string(static_cast<int>(obs.roll_motion_deg)) + "° < " + std::to_string(static_cast<int>(cfg_.min_roll_motion_deg)) + "°)，标定 roll 不可靠，将用先验 0°");
        if (obs.pitch_motion_deg < cfg_.min_pitch_motion_deg)
            obs.warnings.push_back("pitch 激励不足 (" + std::to_string(static_cast<int>(obs.pitch_motion_deg)) + "° < " + std::to_string(static_cast<int>(cfg_.min_pitch_motion_deg)) + "°)，标定 pitch 不可靠，将用先验 0°");
        if (obs.yaw_motion_deg < cfg_.min_yaw_motion_deg)
            obs.warnings.push_back("yaw 激励不足 (" + std::to_string(static_cast<int>(obs.yaw_motion_deg)) + "° < " + std::to_string(static_cast<int>(cfg_.min_yaw_motion_deg)) + "°)，建议增加转弯或绕竖轴旋转");
        if (obs.is_planar_motion)
            obs.recommendation = "平面运动：仅对激励充足的自由度（通常为 yaw）精确标定，roll/pitch 用先验 0°；平移 z 可固定或弱约束";
        else
            obs.recommendation = "运动较充分，可对全自由度标定";
    }
    return obs;
}

// ===================================================================
// 按激励施加旋转先验：激励不足的 RPY 分量置 0，仅保留激励充足的分量
// ===================================================================
Sophus::SO3d IMULiDARCalibrator::apply_excitation_prior_to_rotation(
    const Sophus::SO3d& R_handeye,
    const ObservabilityDiagnosis& obs) const {

    if (!cfg_.use_planar_prior) return R_handeye;

    const Eigen::Matrix3d R = R_handeye.matrix();
    double roll_rad = std::atan2(R(2, 1), R(2, 2));
    double pitch_rad = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    double yaw_rad = std::atan2(R(1, 0), R(0, 0));

    double roll_prior = (obs.roll_motion_deg >= cfg_.min_roll_motion_deg) ? roll_rad : 0.0;
    double pitch_prior = (obs.pitch_motion_deg >= cfg_.min_pitch_motion_deg) ? pitch_rad : 0.0;
    double yaw_prior = (obs.yaw_motion_deg >= cfg_.min_yaw_motion_deg) ? yaw_rad : yaw_rad;  // 车辆通常 yaw 充足，始终保留

    // 从 RPY(roll, pitch, yaw) 重构成旋转矩阵：R = Rz(yaw)*Ry(pitch)*Rx(roll)
    const double cr = std::cos(roll_prior), sr = std::sin(roll_prior);
    const double cp = std::cos(pitch_prior), sp = std::sin(pitch_prior);
    const double cy = std::cos(yaw_prior), sy = std::sin(yaw_prior);
    Eigen::Matrix3d R_new;
    R_new(0, 0) = cy * cp;   R_new(0, 1) = cy * sp * sr - sy * cr;   R_new(0, 2) = cy * sp * cr + sy * sr;
    R_new(1, 0) = sy * cp;   R_new(1, 1) = sy * sp * sr + cy * cr;   R_new(1, 2) = sy * sp * cr - cy * sr;
    R_new(2, 0) = -sp;       R_new(2, 1) = cp * sr;                  R_new(2, 2) = cp * cr;
    return Sophus::SO3d(R_new);
}

}  // namespace ns_unicalib
