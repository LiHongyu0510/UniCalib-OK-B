#pragma once
/**
 * UniCalib Unified — 相机内参标定
 * 支持: 针孔 (OpenCV radtan) + 鱼眼等距 (OpenCV fisheye/kb4)
 * 输入: 棋盘格图像序列 / 圆形标定板
 * 来源: unicalib_C_plus_plus/camera_fisheye.hpp + OpenCV calib3d
 */

#include "unicalib/common/calib_param.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <vector>
#include <functional>
#include <optional>

namespace ns_unicalib {

// ===================================================================
// 标定板配置
// ===================================================================
struct TargetConfig {
    enum class Type { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES } type;

    int cols = 9;             // 内角点列数 (棋盘) / 列数 (圆点)
    int rows = 6;             // 内角点行数
    double square_size_m = 0.025;  // 方格尺寸 [m]

    // 每个角点的3D世界坐标 (在标定板平面上)
    std::vector<cv::Point3f> object_points() const;
};

// ===================================================================
// 单帧检测结果
// ===================================================================
struct CalibFrame {
    std::string image_path;
    cv::Mat     image;          // 原始图像 (可选, 若已加载)
    std::vector<cv::Point2f> corners;   // 检测到的角点
    bool detection_ok = false;
    double reprojection_error = 0.0;    // 该帧重投影误差
};

// ===================================================================
// 相机内参标定器
// ===================================================================
class CameraIntrinsicCalibrator {
public:
    struct Config {
        TargetConfig target;
        CameraIntrinsics::Model model = CameraIntrinsics::Model::PINHOLE;

        int    max_images    = 100;     // 最多使用图像数
        int    min_images    = 10;      // 最少有效图像数
        int    frame_skip    = 1;       // 每隔 N 帧取一帧
        bool   refine_corners = true;   // 亚像素角点精化
        int    subpix_max_iter = 30;   // 亚像素迭代次数；精度优先可 50
        double subpix_epsilon  = 0.01;  // 亚像素收敛精度；精度优先可 0.001
        double outlier_rejection_sigma = 0.0;  // 离群帧剔除：>0 时剔除重投影误差>mean+σ*std 的帧后重新标定，如 2.0
        int    calibrate_max_iter = 100;       // calibrateCamera 最大迭代次数；精度优先可 200
        double calibrate_epsilon  = 1e-7;      // calibrateCamera 收敛精度；精度优先可 1e-8

        // 针孔畸变模型选项
        bool   fix_k3 = false;
        bool   fix_k4 = false;
        bool   fix_k5 = false;
        bool   fix_k6 = false;
        bool   rational_model = false;   // 使用有理数模型 (k4-k6)

        // 鱼眼模型选项
        bool   fix_skew = true;
        bool   recompute_extrinsic = true;

        double max_rms_px = 1.5;        // 最大可接受重投影误差 [px]
        bool   verbose = true;
        bool   checkerboard_optional = false;  // 为 true 时棋盘格不足仅打 WARNING 不标 ERROR（与 prefer_no_checkerboard 配合）
    };

    explicit CameraIntrinsicCalibrator(const Config& cfg) : cfg_(cfg) {}
    CameraIntrinsicCalibrator() : cfg_(Config{}) {}

    // 主标定函数 — 输入图像路径列表
    std::optional<CameraIntrinsics> calibrate(
        const std::vector<std::string>& image_paths);

    // 主标定函数 — 输入 cv::Mat 列表
    std::optional<CameraIntrinsics> calibrate(
        const std::vector<cv::Mat>& images, int width, int height);

    // 检测单帧角点
    CalibFrame detect_corners(const cv::Mat& image);

    // 单目标定
    std::optional<CameraIntrinsics> calibrate_pinhole(
        const std::vector<CalibFrame>& frames, int width, int height);
    std::optional<CameraIntrinsics> calibrate_fisheye(
        const std::vector<CalibFrame>& frames, int width, int height);

    // 获取所有帧的检测结果 (上次标定)
    const std::vector<CalibFrame>& get_frames() const { return frames_; }

    // 绘制每帧的角点检测结果并保存
    void save_corner_images(const std::string& output_dir);

    // 绘制重投影误差直方图
    void save_reproj_error_plot(const CameraIntrinsics& intrin,
                                 const std::string& output_path);

    using ProgressCallback = std::function<void(int current, int total)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = cb; }

private:
    Config cfg_;
    std::vector<CalibFrame> frames_;
    ProgressCallback progress_cb_;

    // 计算每帧重投影误差
    double compute_reproj_error(
        const CameraIntrinsics& intrin,
        const std::vector<cv::Point3f>& obj_pts,
        const std::vector<cv::Point2f>& img_pts,
        const cv::Mat& rvec, const cv::Mat& tvec);
};

// ===================================================================
// 立体相机外参标定 (Camera-Camera, 基于棋盘格)
// ===================================================================
class StereoCameraCalibrator {
public:
    struct Config {
        TargetConfig target;
        double max_rms_px = 2.0;
        bool   fix_intrinsics = true;   // 固定已知内参，仅优化外参
        bool   verbose = true;
    };

    explicit StereoCameraCalibrator(const Config& cfg) : cfg_(cfg) {}
    StereoCameraCalibrator() : cfg_(Config{}) {}

    // 输入: 两相机同步图像对 + 已知内参
    // 输出: T_cam1_in_cam0 (cam1 在 cam0 坐标系下的变换)
    std::optional<ExtrinsicSE3> calibrate(
        const std::vector<std::string>& images_cam0,
        const std::vector<std::string>& images_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const std::string& cam0_id = "cam0",
        const std::string& cam1_id = "cam1");

private:
    Config cfg_;
};

}  // namespace ns_unicalib
