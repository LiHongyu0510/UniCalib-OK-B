#pragma once
/**
 * UniCalib Unified — 标定参数管理器
 * 统一存储 IMU内参 / 相机内参 / 外参(SE3) / 时间偏移
 * 支持 cereal 序列化 (YAML / JSON)
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <map>
#include <string>
#include <optional>
#include <memory>
#include <vector>
#include <cereal/cereal.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/vector.hpp>

namespace ns_unicalib {

// ===================================================================
// IMU 内参
// ===================================================================
struct IMUIntrinsics {
    using Ptr = std::shared_ptr<IMUIntrinsics>;

    // 陀螺仪
    Eigen::Matrix3d Ma_gyro = Eigen::Matrix3d::Identity();  // 比例因子+交轴误差矩阵
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();     // 零偏
    double noise_gyro = 1e-3;        // rad/s/sqrt(Hz) — 角速度随机游走
    double bias_instab_gyro = 1e-5;  // rad/s — 零偏不稳定性

    // 加速度计
    Eigen::Matrix3d Ma_acce = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bias_acce = Eigen::Vector3d::Zero();
    double noise_acce = 1e-2;        // m/s^2/sqrt(Hz)
    double bias_instab_acce = 1e-4;  // m/s^2

    // 坐标系旋转 (Body -> Gyro坐标系)
    Sophus::SO3d SO3_AtoG = Sophus::SO3d();

    // IMU 与车体外参 T_Body_Imu: 车体系 -> IMU 系 (默认单位矩阵，即重合)
    // 用于描述 IMU 在车体坐标系下的位姿
    Eigen::Matrix4d T_Body_Imu = Eigen::Matrix4d::Identity();

    // 质量指标
    double allan_fit_rms = 0.0;
    int    num_samples_used = 0;

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(noise_gyro), CEREAL_NVP(bias_instab_gyro),
           CEREAL_NVP(noise_acce), CEREAL_NVP(bias_instab_acce),
           CEREAL_NVP(allan_fit_rms), CEREAL_NVP(num_samples_used));
    }
};

// ===================================================================
// 相机内参 (通用)
// ===================================================================
struct CameraIntrinsics {
    using Ptr = std::shared_ptr<CameraIntrinsics>;

    enum class Model { PINHOLE, FISHEYE } model = Model::PINHOLE;

    int width = 0, height = 0;

    // 焦距与主点
    double fx = 0, fy = 0;
    double cx = 0, cy = 0;

    // 畸变系数
    // PINHOLE: k1, k2, p1, p2, [k3, k4, k5, k6] (OpenCV)
    // FISHEYE: k1, k2, k3, k4 (equidistant / kb4)
    std::vector<double> dist_coeffs;

    // 重投影误差 (标定质量指标)
    double rms_reproj_error = 0.0;
    int    num_images_used = 0;

    // 内参矩阵
    Eigen::Matrix3d K() const {
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0,0) = fx; K(1,1) = fy;
        K(0,2) = cx; K(1,2) = cy;
        return K;
    }

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(width), CEREAL_NVP(height),
           CEREAL_NVP(fx), CEREAL_NVP(fy),
           CEREAL_NVP(cx), CEREAL_NVP(cy),
           CEREAL_NVP(dist_coeffs),
           CEREAL_NVP(rms_reproj_error),
           CEREAL_NVP(num_images_used));
    }
};

// ===================================================================
// 外参 — SE3 变换 T_TargetInRef (target 在 reference 坐标系下)
// ===================================================================
struct ExtrinsicSE3 {
    using Ptr = std::shared_ptr<ExtrinsicSE3>;

    std::string ref_sensor_id;      // 参考传感器 (e.g. "imu_0")
    std::string target_sensor_id;   // 目标传感器 (e.g. "lidar_front")

    Sophus::SO3d SO3_TargetInRef;   // 旋转: target -> ref
    Eigen::Vector3d POS_TargetInRef = Eigen::Vector3d::Zero();  // 平移 [m]
    double time_offset_s = 0.0;     // t_ref = t_target + time_offset_s [s]

    // 协方差 (可选)
    std::optional<Eigen::Matrix<double,6,6>> covariance;

    // 质量指标
    double residual_rms = 0.0;
    bool   is_converged = false;
    // IMU-LiDAR 标定精度（仅该类型外参使用，<0 表示未设置）
    double handeye_rot_residual_deg = -1.0;  // 手眼旋转平均残差 [deg]
    double trans_rms_m_s = -1.0;            // 平移估计 RMS [m/s]
    double bspline_final_cost = -1.0;       // B样条优化最终代价

    // 获取 SE3 变换矩阵
    Sophus::SE3d SE3_TargetInRef() const {
        return Sophus::SE3d(SO3_TargetInRef, POS_TargetInRef);
    }
    // 设置 SE3
    void set_SE3(const Sophus::SE3d& T) {
        SO3_TargetInRef = T.so3();
        POS_TargetInRef = T.translation();
    }
    // 获取 4×4 矩阵
    Eigen::Matrix4d T_mat() const {
        return SE3_TargetInRef().matrix();
    }
    // 获取 RPY [deg]
    Eigen::Vector3d euler_deg() const;
    // 获取平移 [m]
    Eigen::Vector3d translation() const { return POS_TargetInRef; }

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(ref_sensor_id), CEREAL_NVP(target_sensor_id),
           CEREAL_NVP(time_offset_s),
           CEREAL_NVP(residual_rms), CEREAL_NVP(is_converged));
        // SO3/Eigen 由 cereal_eigen_include 处理
    }
};

// ===================================================================
// 全局标定参数管理器
// ===================================================================
class CalibParamManager {
public:
    using Ptr = std::shared_ptr<CalibParamManager>;

    // IMU 内参 — key: sensor_id
    std::map<std::string, IMUIntrinsics::Ptr>      imu_intrinsics;

    // 相机内参 — key: sensor_id
    std::map<std::string, CameraIntrinsics::Ptr>   camera_intrinsics;

    // 外参 (SE3) — key: "ref_id->target_id"
    std::map<std::string, ExtrinsicSE3::Ptr>        extrinsics;

    // 重力向量 (世界系)
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

    // ---------------------------------------------------------------
    // 工厂方法
    static Ptr Create() { return std::make_shared<CalibParamManager>(); }

    // ---------------------------------------------------------------
    // 外参访问
    static std::string extrinsic_key(const std::string& ref, const std::string& target) {
        return ref + "->" + target;
    }
    ExtrinsicSE3::Ptr get_extrinsic(
        const std::string& ref, const std::string& target) const;
    ExtrinsicSE3::Ptr get_or_create_extrinsic(
        const std::string& ref, const std::string& target);

    // ---------------------------------------------------------------
    // 序列化
    void save_yaml(const std::string& path) const;
    void save_json(const std::string& path) const;
    void load_yaml(const std::string& path);
    void load_json(const std::string& path);

    // ---------------------------------------------------------------
    // 打印摘要
    void print_summary() const;
};

}  // namespace ns_unicalib
