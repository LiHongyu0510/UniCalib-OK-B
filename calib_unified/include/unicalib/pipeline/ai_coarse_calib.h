#pragma once
/**
 * UniCalib Unified — AI 粗标定适配器
 *
 * 统一封装各 AI 模型的调用接口:
 *
 *   DMCalibAdapter        → DM-Calib (扩散模型)  → 相机内参粗估
 *     原理: 单张图像 → Stable Diffusion → Camera Image → RANSAC → 内参
 *     调用: Python subprocess (DMCalib/tools/infer_unicalib.py)
 *
 *   MIASLCECAdapter       → MIAS-LCEC            → LiDAR-Cam 外参粗估
 *     原理: 跨模态掩码匹配 (C3M), MobileSAM + 点云特征
 *     调用: Python subprocess (MIAS-LCEC 工具链)
 *
 *   TransformerIMUAdapter → Transformer-IMU-Calibrator → IMU 内参粗估
 *     原理: Transformer 网络估计 IMU 动态标定参数
 *     调用: Python subprocess (eval.py)
 *
 *   L2CalibAdapter        → learn-to-calibrate (L2Calib) → IMU-LiDAR 外参粗估
 *     原理: SE(3)-流形强化学习 (PPO + Bingham策略)
 *     调用: Python subprocess (train.py with small epochs for inference)
 *
 * 设计原则:
 *   - 所有 AI 调用通过 Python subprocess 隔离, C++ 主程序不依赖 PyTorch
 *   - 标准化输入/输出格式 (YAML/JSON)
 *   - 每次调用有独立日志文件
 *   - 超时保护 + 错误恢复
 *   - 若 AI 模型不可用, 自动降级到传统方法
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

namespace ns_unicalib {

// ===========================================================================
// AI 模型调用结果基类
// ===========================================================================
struct AICoarseResult {
    bool  success = false;
    double confidence = 0.0;    // 0~1, AI 模型置信度
    std::string model_name;
    std::string log_path;
    double elapsed_ms = 0.0;
    std::string error_msg;
    /** 为 true 表示环境/依赖错误（如 NumPy 版本不兼容），调用方应停止重试并回退 */
    bool env_error = false;

    bool is_reliable(double min_conf = 0.5) const {
        return success && confidence >= min_conf;
    }
};

// ===========================================================================
// DM-Calib 适配器 — 相机内参粗估
// ===========================================================================
struct DMCalibResult : AICoarseResult {
    CameraIntrinsics coarse_intrin;
    // 原始预测的 Camera Image (调试用)
    std::string camera_image_path;
};

class DMCalibAdapter {
public:
    struct Config {
        std::string python_exe      = "python3";
        std::string repo_dir;                          // DM-Calib 根目录
        std::string model_path;                        // 预训练模型路径
        std::string infer_script    = "DMCalib/tools/infer_unicalib.py";
        std::string domain          = "outdoor";       // outdoor/indoor
        bool        scale_10        = true;
        bool        domain_specify  = true;
        bool        use_cpu         = true;            // true: 强制 CPU 推理（GPU 架构不兼容时可先用 CPU）
        int         timeout_sec     = 600;             // 单次推理超时(秒)；CPU 推理较慢建议 600+，可配 third_party.dm_calib_timeout_sec
        std::string work_dir        = "/tmp/dmcalib_work";
        // CPU 加速：>0 时传给 Python，否则用脚本默认。denoise_steps 减小/ensemble_size 减小/processing_res 减小可显著加速，略损精度
        int         denoise_steps   = 0;               // 0=默认 20；CPU 可设 10～12
        int         ensemble_size  = 0;                // 0=默认 3；CPU 可设 1
        int         processing_res  = 0;                // 0=默认 768；CPU 可设 512
        // 是否使用缓存 (相同图像路径跳过推理)
        bool        use_cache       = true;
    };

    explicit DMCalibAdapter(const Config& cfg) : cfg_(cfg) {}

    // 从单张图像估计内参
    DMCalibResult estimate(const std::string& image_path,
                           const std::string& output_dir = "") const;

    // 批量估计 (多帧投票取中位数)
    DMCalibResult estimate_multi(const std::vector<std::string>& image_paths,
                                  const std::string& output_dir = "") const;

    // 检查模型是否可用
    bool is_available() const;

private:
    Config cfg_;

    std::string build_cmd(const std::string& input_dir,
                          const std::string& output_dir) const;
    CameraIntrinsics parse_output(const std::string& output_json_path,
                                  int img_w, int img_h) const;
    void log_call(const std::string& cmd,
                  int exit_code, double elapsed_ms) const;
};

// ===========================================================================
// MIAS-LCEC 适配器 — LiDAR-Camera 外参粗估
// ===========================================================================
struct MIASLCECResult : AICoarseResult {
    ExtrinsicSE3 coarse_extrin;
    // 可视化结果路径
    std::string vis_path;
    // C3M 匹配对数量
    int num_correspondences = 0;
};

class MIASLCECAdapter {
public:
    struct Config {
        std::string python_exe      = "python3";
        std::string repo_dir;                          // MIAS-LCEC 根目录
        std::string calib_script    = "src/calib_main.py";
        std::string model_path;                        // 可选：Overlap Transformer 等模型路径 (.pth.tar)，有则传 --model_path 优先深度模型
        bool        allow_pnp_fallback = true;          // 推理不可用时允许回退 PnP（传 --no-require-model）
        int         timeout_sec     = 180;
        std::string work_dir        = "/tmp/mias_work";
        // 传感器配置
        std::string lidar_type      = "velodyne";      // velodyne/livox/ouster
        std::string camera_model    = "pinhole";       // pinhole/fisheye

        // ─── C3M / SAM-only 迭代精化配置（通过 run_lidar_cam_coarse.py 透传到 Python） ───
        // 若为 true：在 SAM-only 粗标定路径上启用 iterative refine（PnP→重新聚合→PnP）
        bool   c3m_use_iterative_refine = false;
        int    c3m_iter_max = 0;           // 0=脚本默认（当前默认 3）
        double c3m_iter_thresh = 0.0;      // 0=脚本默认（当前默认 0.01）
        double c3m_similarity_threshold = 0.0; // 0=脚本默认（当前默认 0.7）

        // ─── 粗标定质量评估阈值（写入 quality_report.yaml，供 C++ 管线/人工诊断） ───
        double quality_ncc_good = 0.3;
        double quality_ncc_acceptable = 0.2;
        double quality_rms_good_px = 2.0;
        double quality_rms_acceptable_px = 5.0;
        double quality_inlier_ratio_good = 0.6;
        double quality_inlier_ratio_acceptable = 0.4;
    };

    explicit MIASLCECAdapter(const Config& cfg) : cfg_(cfg) {}

    // 从点云文件 + 图像文件估计外参
    MIASLCECResult estimate(
        const std::string& pcd_file,
        const std::string& image_file,
        const CameraIntrinsics& cam_intrin,
        const std::string& output_dir = "") const;

    // 检查模型是否可用
    bool is_available() const;

private:
    Config cfg_;

    std::string write_sensor_config(const CameraIntrinsics& intrin,
                                     const std::string& work_dir) const;
    ExtrinsicSE3 parse_output(const std::string& result_yaml) const;
};

// ===========================================================================
// Transformer-IMU 适配器 — IMU 内参粗估
// ===========================================================================
struct TransformerIMUResult : AICoarseResult {
    IMUIntrinsics coarse_intrin;
    // 网络输出的 offset/drift 序列
    std::string raw_output_path;
};

class TransformerIMUAdapter {
public:
    struct Config {
        std::string python_exe      = "python3";
        std::string repo_dir;                          // Transformer-IMU-Calibrator 根目录
        std::string eval_script     = "eval_unicalib.py";  // UniCalib 入口：--imu_data/--weights/--output_dir
        std::string model_weights   = "model/TIC_13.pth"; // 与仓库 model/TIC_13.pth 一致，确保加载
        int         timeout_sec     = 60;
        std::string work_dir        = "/tmp/transformer_imu_work";
        bool        use_mean_only   = false;           // 单 IMU 时传 --mean-only，避免 6× 复制影响精度
    };

    explicit TransformerIMUAdapter(const Config& cfg) : cfg_(cfg) {}

    // 从 IMU 数据序列估计内参 (bias/scale)
    TransformerIMUResult estimate(
        const std::vector<IMUFrame>& imu_data,
        const std::string& output_dir = "") const;

    // 检查模型/脚本是否可用（有 repo + 脚本 + 权重 + torch 时为 true）
    bool is_available() const;

    /** 纯 C++ 零偏估计（与 eval_unicalib.py 当前逻辑等价：均值零偏，无模型推理）。不依赖 Python。 */
    static TransformerIMUResult estimate_native(const std::vector<IMUFrame>& imu_data);

private:
    Config cfg_;

    std::string serialize_imu_data(const std::vector<IMUFrame>& data,
                                    const std::string& path) const;
    IMUIntrinsics parse_output(const std::string& result_json) const;
};

// ===========================================================================
// L2Calib (learn-to-calibrate) 适配器 — IMU-LiDAR 外参粗估
// ===========================================================================
struct L2CalibResult : AICoarseResult {
    ExtrinsicSE3 coarse_extrin;
    double final_reward = 0.0;   // RL agent 最终奖励值
    int    num_epochs = 0;
    std::string trajectory_path;
};

class L2CalibAdapter {
public:
    struct Config {
        std::string python_exe      = "python3";
        std::string repo_dir;                          // learn-to-calibrate 根目录
        std::string train_script    = "train.py";
        int         num_epochs      = 500;             // 粗估时用较少 epoch
        std::string alg             = "ppo";
        std::string so3_dist        = "Bingham";
        double      rough_trans_m   = 0.1;             // 平移粗估 [m]
        int         timeout_sec     = 300;
        std::string work_dir        = "/tmp/l2calib_work";
        // LIO 配置 (FastLIO)
        std::string fastlio_config  = "";
        std::string lidar_type      = "velodyne";
        std::string imu_topic       = "/imu/data";
    };

    explicit L2CalibAdapter(const Config& cfg) : cfg_(cfg) {}

    // 从 ROS bag 文件估计外参
    L2CalibResult estimate_from_bag(
        const std::string& bag_path,
        const std::string& output_dir = "") const;

    // 从轨迹文件直接估计 (跳过 LIO)
    L2CalibResult estimate_from_trajectories(
        const std::string& lidar_traj_path,
        const std::string& imu_traj_path,
        const std::string& output_dir = "") const;

    // 检查模型是否可用
    bool is_available() const;

private:
    Config cfg_;

    ExtrinsicSE3 parse_output(const std::string& result_yaml) const;
};

// ===========================================================================
// 统一 AI 粗标定管理器
// ===========================================================================
class AICoarseCalibManager {
public:
    struct Config {
        std::string ai_root;         // AI 工程根目录 (包含所有子目录)
        std::string python_exe       = "python3";
        std::string work_dir         = "/tmp/unicalib_ai_work";

        DMCalibAdapter::Config        dm_calib;
        MIASLCECAdapter::Config       mias_lcec;
        TransformerIMUAdapter::Config  transformer_imu;
        L2CalibAdapter::Config        l2calib;

        // 置信度阈值 (低于此值时降级到传统方法)
        double min_confidence_threshold = 0.4;
        // 是否在 AI 失败时自动降级
        bool fallback_on_failure = true;
    };

    explicit AICoarseCalibManager(const Config& cfg);

    // -----------------------------------------------------------------------
    // 各传感器粗标定接口
    // -----------------------------------------------------------------------

    // 相机内参粗估 (DM-Calib)
    std::optional<CameraIntrinsics> coarse_cam_intrinsic(
        const std::vector<std::string>& image_paths,
        int img_w, int img_h,
        const std::string& cam_id = "cam_0") const;

    // IMU 内参粗估 (Transformer-IMU-Calibrator)
    std::optional<IMUIntrinsics> coarse_imu_intrinsic(
        const std::vector<IMUFrame>& imu_data,
        const std::string& imu_id = "imu_0") const;

    // IMU-LiDAR 外参粗估 (L2Calib)
    std::optional<ExtrinsicSE3> coarse_imu_lidar(
        const std::string& bag_path,
        const std::string& imu_id   = "imu_0",
        const std::string& lidar_id = "lidar_0") const;

    // LiDAR-Camera 外参粗估 (MIAS-LCEC)
    std::optional<ExtrinsicSE3> coarse_lidar_cam(
        const std::string& pcd_file,
        const std::string& image_file,
        const CameraIntrinsics& cam_intrin,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0") const;

    // -----------------------------------------------------------------------
    // 可用性检查
    // -----------------------------------------------------------------------
    bool check_dm_calib()        const;
    bool check_mias_lcec()       const;
    bool check_transformer_imu() const;
    bool check_l2calib()         const;

    void print_availability() const;

private:
    Config cfg_;
    std::unique_ptr<DMCalibAdapter>        dm_calib_;
    std::unique_ptr<MIASLCECAdapter>       mias_lcec_;
    std::unique_ptr<TransformerIMUAdapter> transformer_imu_;
    std::unique_ptr<L2CalibAdapter>        l2calib_;

    void setup_adapters();
};

}  // namespace ns_unicalib
