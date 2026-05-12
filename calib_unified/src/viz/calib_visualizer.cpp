/**
 * UniCalib Unified — 统一标定可视化系统
 *
 * 完整实现:
 *   1. IMU-LiDAR 标定可视化 (LiDAR里程计、旋转对、B样条收敛)
 *   2. LiDAR-Camera 投影验证 (边缘对齐、互信息)
 *   3. 通用可视化 (收敛曲线、时间偏移图、误差分布)
 *   4. Pangolin 3D 可视化（交互式显示）
 *
 * 工程化改造: 使用统一的异常体系
 */

#include "unicalib/viz/calib_visualizer.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/error_code.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#if defined(__linux__) || defined(__APPLE__)
#include <stdlib.h>  // setenv (POSIX)
#endif

// Pangolin (可选，如果编译失败则仅使用OpenCV)
// #include <pangolin/pangolin.h>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===========================================================================
// 构造/析构
// ===========================================================================
CalibVisualizer::CalibVisualizer(const VizConfig& config)
    : config_(config), stopped_(false), id_counter_(0), history_index_(-1) {

    // 创建日志器
    logger_ = spdlog::get("unicalib");
    if (!logger_) {
        logger_ = spdlog::stdout_color_mt("unicalib");
    }

    // 创建报告生成器
    report_generator_ = std::make_shared<ReportGenerator>();

    // 初始化 Pangolin 渲染器 (仅在启用 Pangolin 时)
#if UNICALIB_WITH_PANGOLIN
    try {
        pangolin_render_ = std::make_shared<PangolinRender>();
    } catch (const std::exception& e) {
        UNICALIB_WARN("[CalibVisualizer] Pangolin 渲染器初始化失败: {}", e.what());
        pangolin_render_ = nullptr;
    }
#endif

    // 创建截图目录
    if (!config_.screenshot_dir.empty()) {
        std::error_code ec;
        fs::create_directories(config_.screenshot_dir, ec);
        if (ec) {
            UNICALIB_THROW_SYSTEM(ErrorCode::FILE_WRITE_ERROR,
                "CalibVisualizer 无法创建截图目录: " + config_.screenshot_dir + " (" + ec.message() + ")");
        }
    }

    UNICALIB_INFO("[CalibVisualizer] 初始化完成");
}

CalibVisualizer::~CalibVisualizer() {
    stopped_ = true;
    if (cloud_viewer_) {
        cloud_viewer_.reset();
    }
}

// ===========================================================================
// 进度报告
// ===========================================================================
void CalibVisualizer::report_progress(const std::string& stage,
                                       const std::string& step,
                                       double progress,
                                       const std::string& message) {
    if (progress_cb_) {
        progress_cb_(stage, step, progress, message);
    }
}

// ===========================================================================
// 3D 可视化接口
// ===========================================================================
CloudViewer::Ptr CalibVisualizer::get_cloud_viewer() {
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }
    return cloud_viewer_;
}

// ===========================================================================
// IMU-LiDAR 标定可视化
// ===========================================================================
void CalibVisualizer::show_imu_lidar_result(const IMULiDARVizData& data, bool block) {
    (void)block;
    if (!cloud_viewer_) {
        try {
            cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
        } catch (const std::exception& e) {
            UNICALIB_THROW_SYSTEM(ErrorCode::DISPLAY_INIT_FAILED,
                "CalibVisualizer 创建 CloudViewer 失败: " + std::string(e.what()));
        }
    }

    // 清除之前的可视化
    cloud_viewer_->clear();

    // 1. 显示 LiDAR 里程计轨迹 (add_trajectory 需要 vector<SE3d>)
    if (!data.lidar_odom_poses.empty()) {
        std::vector<Sophus::SE3d> poses_se3;
        poses_se3.reserve(data.lidar_odom_poses.size());
        for (const auto& p : data.lidar_odom_poses)
            poses_se3.push_back(p.second);
        cloud_viewer_->add_trajectory(poses_se3, "lidar_odom", config_.lidar_trajectory_color, 3.0f);
        for (size_t i = 0; i < data.lidar_odom_poses.size(); i += 10) {
            cloud_viewer_->add_coordinate_frame(data.lidar_odom_poses[i].second, "odom_" + std::to_string(i), 0.3f);
        }
    }

    // 2. 显示旋转对可视化
    if (!data.rotation_pairs.empty()) {
        show_rotation_pairs(data.rotation_pairs, true);
    }

    // 3. 显示 B样条收敛曲线
    if (!data.optimization_history.empty()) {
        show_bspline_optimization(data.optimization_history, config_.enable_animation);
    }

    // 4. 显示最终外参坐标系
    if (data.coarse_result.has_value() || data.fine_result.has_value()) {
        const auto& result = data.fine_result.has_value() ? data.fine_result : data.coarse_result;
        cloud_viewer_->add_coordinate_frame(Sophus::SE3d(result->SO3_TargetInRef, result->POS_TargetInRef),
                                            "extrinsic_final", 0.5f);
    }

    // 5. 保存可视化
    save_plots(config_.screenshot_dir + "/imu_lidar_result");

    report_progress("IMU-LiDAR", "visualization_complete", 1.0, "Visualization complete");
}

void CalibVisualizer::show_rotation_pairs(const std::vector<IMULiDARVizData::RotationPair>& pairs, bool show_imu_traj) {
    if (!cloud_viewer_ || pairs.empty()) return;

    // 清除旋转对可视化
    cloud_viewer_->clear("rot_pair");

    // 创建 IMU 轨迹
    std::vector<Sophus::SE3d> imu_poses;
    std::vector<Sophus::SE3d> lidar_poses;
    for (const auto& pair : pairs) {
        imu_poses.emplace_back(pair.rot_imu, Eigen::Vector3d::Zero());
        lidar_poses.emplace_back(pair.rot_lidar, Eigen::Vector3d::Zero());
    }

    // 添加轨迹
    if (show_imu_traj && !imu_poses.empty()) {
        cloud_viewer_->add_trajectory(imu_poses, "imu_traj_rot", config_.imu_trajectory_color, 2.0f);
    }
    cloud_viewer_->add_trajectory(lidar_poses, "lidar_traj_rot", config_.lidar_trajectory_color, 2.0f);

    // 为每对旋转添加可视化
    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& pair = pairs[i];

        // IMU 旋转轴 (红色)
        Eigen::Vector3d axis_imu(1, 0, 0);
        cloud_viewer_->add_coordinate_frame(Sophus::SE3d(pair.rot_imu, axis_imu * 0.2),
                                             "imu_axis_" + std::to_string(i), 0.2f);

        // LiDAR 旋转轴 (蓝色)
        Eigen::Vector3d axis_lidar(1, 0, 0);
        cloud_viewer_->add_coordinate_frame(Sophus::SE3d(pair.rot_lidar, axis_lidar * 0.2),
                                             "lidar_axis_" + std::to_string(i), 0.2f);
    }

    if (config_.enable_animation) {
        cloud_viewer_->spin_once(100);
    }
}

void CalibVisualizer::show_lidar_odometry(const std::vector<Sophus::SE3d>& poses) {
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }

    cloud_viewer_->clear();
    cloud_viewer_->add_trajectory(poses, "lidar_odom", config_.lidar_trajectory_color, 3.0f);

    // 添加坐标轴
    for (size_t i = 0; i < poses.size(); i += 5) {
        cloud_viewer_->add_coordinate_frame(poses[i], "odom_" + std::to_string(i), 0.5f);
    }

    save_plots(config_.screenshot_dir + "/lidar_odom");
}

void CalibVisualizer::show_bspline_optimization(const std::vector<IMULiDARVizData::OptimizationLog>& history, bool show_animation) {
    (void)show_animation;
    if (history.empty()) return;

    std::vector<double> iterations;
    std::vector<double> costs;
    std::vector<double> time_offsets;
    for (size_t i = 0; i < history.size(); ++i) {
        iterations.push_back(static_cast<double>(i));
        costs.push_back(history[i].cost);
        for (const auto& ts : history[i].time_offsets) {
            time_offsets.push_back(ts);
        }
    }

    cv::Mat conv_curve = draw_optimization_convergence_curve(iterations, costs, "B-spline Optimization",
                                                              config_.plot_width, config_.plot_height);
    save_plots(config_.screenshot_dir + "/bspline_convergence");

    if (!time_offsets.empty() && time_offsets.size() == costs.size()) {
        cv::Mat time_curve = plot_time_offset_convergence_curve(time_offsets, costs, "Time Offset Estimation");
        (void)time_curve;
        save_plots(config_.screenshot_dir + "/time_offset");
    }

    if (cloud_viewer_) {
        cloud_viewer_->spin_once(100);
    }
}

// ===========================================================================
// LiDAR-Camera 投影可视化
// ===========================================================================
void CalibVisualizer::show_lidar_camera_projection(const std::vector<LiDARScan>& lidar_scans,
                                                    const std::vector<CameraFrame>& camera_frames,
                                                    const ExtrinsicSE3& extrinsic,
                                                    const CameraIntrinsics& intrinsics,
                                                    bool save_images) {
    if (lidar_scans.empty() || camera_frames.empty()) return;

    fs::create_directories(config_.screenshot_dir + "/lidar_cam_projection");

    Sophus::SE3d T_lidar_cam = extrinsic.SE3_TargetInRef().inverse();

    for (size_t i = 0; i < std::min(lidar_scans.size(), camera_frames.size()); ++i) {
        auto& scan = lidar_scans[i];
        auto& frame = camera_frames[i];

        if (!scan.cloud || scan.cloud->empty()) continue;

        // 投影点云到图像
        cv::Mat proj_img = LiDARProjectionViz::project_to_image(
            *scan.cloud, frame.image, T_lidar_cam,
            intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
            intrinsics.width, intrinsics.height, 0.5, 50.0);

        if (save_images) {
            std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            std::string proj_path = config_.screenshot_dir + "/lidar_cam_projection/" +
                                     std::to_string(i) + "_" + timestamp + ".png";
            cv::imwrite(proj_path, proj_img);
        }
    }

    report_progress("LiDAR-Camera", "projection_complete", 1.0, "Projection visualization complete");
}

void CalibVisualizer::show_lidar_projection_on_image(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const cv::Mat& image,
    const Sophus::SE3d& T_cam_lidar,
    const CameraIntrinsics& intrin,
    const std::string& save_path,
    int display_ms) {

    if (!cloud || cloud->empty() || image.empty()) {
        UNICALIB_WARN("[Viz] show_lidar_projection_on_image 跳过: 点云或图像为空");
        return;
    }

    cv::Mat vis = image.clone();
    if (vis.channels() == 1)
        cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);

    const int W = intrin.width > 0 ? intrin.width : vis.cols;
    const int H = intrin.height > 0 ? intrin.height : vis.rows;
    const double fx = intrin.fx, fy = intrin.fy, cx = intrin.cx, cy = intrin.cy;

    double d_min = 1e9, d_max = -1e9;
    for (const auto& pt : cloud->points) {
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T_cam_lidar * p_l;
        if (p_c.z() > 0.1) {
            d_min = std::min(d_min, p_c.z());
            d_max = std::max(d_max, p_c.z());
        }
    }
    if (d_max <= d_min) d_max = d_min + 1.0;

    for (const auto& pt : cloud->points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T_cam_lidar * p_l;
        if (p_c.z() < 0.1) continue;

        double u = fx * p_c.x() / p_c.z() + cx;
        double v = fy * p_c.y() / p_c.z() + cy;
        int iu = static_cast<int>(std::round(u));
        int iv = static_cast<int>(std::round(v));
        if (iu < 2 || iu >= W - 2 || iv < 2 || iv >= H - 2) continue;

        double ratio = (p_c.z() - d_min) / (d_max - d_min);
        ratio = std::max(0.0, std::min(1.0, ratio));
        int r = static_cast<int>((1.0 - ratio) * 255);
        int b = static_cast<int>(ratio * 255);
        cv::circle(vis, cv::Point(iu, iv), 2, cv::Scalar(b, 80, r), -1);
    }

    if (!save_path.empty()) {
        fs::path p(save_path);
        if (p.has_parent_path()) {
            std::error_code ec;
            fs::create_directories(p.parent_path(), ec);
        }
        if (cv::imwrite(save_path, vis))
            UNICALIB_INFO("[Viz] 投影图已保存: {}", save_path);
    }

    const char* win_name = "LiDAR-Camera 投影 (标定结果)";
    cv::imshow(win_name, vis);
    if (display_ms > 0) {
        cv::waitKey(display_ms);
    } else {
        // 一直显示，直到用户关闭窗口或按键
        UNICALIB_INFO("[Viz] 投影窗口已打开，关闭窗口或按任意键继续");
        while (cv::getWindowProperty(win_name, cv::WND_PROP_VISIBLE) > 0) {
            if (cv::waitKey(100) >= 0) break;
        }
    }
}

void CalibVisualizer::show_image_alignment(const std::vector<CameraFrame>& camera_frames,
                                           const ExtrinsicSE3& extrinsic,
                                           const CameraIntrinsics& intrinsics) {
    if (camera_frames.empty()) return;

    fs::create_directories(config_.screenshot_dir + "/image_alignment");

    Sophus::SE3d T_lidar_cam = extrinsic.SE3_TargetInRef().inverse();

    for (size_t i = 0; i < camera_frames.size(); ++i) {
        auto& frame = camera_frames[i];

        // 绘制角点重投影
        std::vector<Eigen::Vector2d> corners_2d;
        std::vector<Eigen::Vector2d> corners_reprojected;
        // 注意: 实际角点检测应在标定阶段完成
        // 这里仅提供可视化框架，实际数据需从标定结果中传入
        UNICALIB_DEBUG("[CalibVisualizer] 角点重投影可视化 (当前为空)");

        cv::Mat alignment_img = draw_camera_reprojection_errors(corners_2d, corners_reprojected,
                                                                  "Image Alignment");

        std::string img_path = config_.screenshot_dir + "/image_alignment/frame_" + std::to_string(i) + ".png";
        cv::imwrite(img_path, alignment_img);
    }
}

// ===========================================================================
// 点云对齐可视化
// ===========================================================================
void CalibVisualizer::show_cloud_alignment(const std::vector<LiDARScan>& scans_before,
                                            const std::vector<LiDARScan>& scans_after,
                                            const ExtrinsicSE3& extrinsic) {
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }

    cloud_viewer_->clear();

    Sophus::SE3d T_extrinsic = extrinsic.SE3_TargetInRef();

    // 变换并显示对齐前的点云 (灰色)
    for (size_t i = 0; i < std::min(scans_before.size(), scans_after.size()); ++i) {
        auto& scan_before = scans_before[i];
        auto& scan_after = scans_after[i];

        if (!scan_before.cloud || !scan_after.cloud) continue;

        // 变换后的点云
        auto cloud_transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(*scan_after.cloud, *cloud_transformed, Eigen::Affine3d(T_extrinsic.matrix()));

        // 添加到可视化器
        cloud_viewer_->add_cloud(cloud_transformed, "cloud_aligned", config_.point_cloud_color, 1.5f);
    }

    // 添加坐标系
    cloud_viewer_->add_coordinate_frame(Sophus::SE3d(), "imu_origin", 0.5f);
    cloud_viewer_->add_coordinate_frame(extrinsic.SE3_TargetInRef(), "extrinsic_frame", 0.5f);

    cloud_viewer_->spin_once(100);

    save_plots(config_.screenshot_dir + "/cloud_comparison");
}

// ===========================================================================
// 保存和报告
// ===========================================================================
void CalibVisualizer::save_calibration_results(const CalibParamManager& params, const std::string& output_dir, const std::string& format) {
    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
        UNICALIB_THROW_SYSTEM(ErrorCode::FILE_WRITE_ERROR,
            "save_calibration_results 无法创建目录: " + output_dir + " (" + ec.message() + ")");
    }

    if (!report_generator_) {
        UNICALIB_THROW_SYSTEM(ErrorCode::INTERNAL_ERROR,
            "save_calibration_results report_generator_ 为空");
    }
    std::string report_path = output_dir + "/calibration_report." + format;
    try {
        report_generator_->generate(report_path);
    } catch (const std::exception& e) {
        UNICALIB_THROW(ErrorCode::REPORT_GENERATION_FAILED,
            "生成标定报告失败 path=" + report_path + " detail=" + e.what());
    }

    // 保存可视化数据
    for (const auto& [key, data] : viz_data_) {
        std::string data_path = output_dir + "/" + key + "_viz_data.json";
        std::ofstream ofs(data_path);
        ofs << "{\n";
        ofs << "  \"key\": \"" << key << "\",\n";
        ofs << "  \"data\": " << data.to_json() << "\n";
        ofs << "}\n";
    }
}

void CalibVisualizer::start_display(const std::string& title, int width, int height) {
#if UNICALIB_WITH_PANGOLIN
    if (pangolin_render_ && !pangolin_render_->is_running()) {
        // 未设置时自动设置 XDG_RUNTIME_DIR，避免 GLFW/Pangolin 报错导致界面异常
#if defined(__linux__) || defined(__APPLE__)
        const char* xdg_runtime = std::getenv("XDG_RUNTIME_DIR");
        if (!xdg_runtime || xdg_runtime[0] == '\0') {
            if (setenv("XDG_RUNTIME_DIR", "/tmp", 0) == 0) {
                UNICALIB_INFO("[Viz/Display] 已自动设置 XDG_RUNTIME_DIR=/tmp（原未设置，避免 GLFW 报错）");
            }
        }
#endif
        // 记录显示环境，便于排查
        const char* display = std::getenv("DISPLAY");
        const char* xdg_now = std::getenv("XDG_RUNTIME_DIR");
        const char* wayland = std::getenv("WAYLAND_DISPLAY");
        UNICALIB_INFO("[Viz/Display] 显示环境 DISPLAY={}  XDG_RUNTIME_DIR={}  WAYLAND_DISPLAY={}",
                      display ? display : "(未设置)",
                      xdg_now ? xdg_now : "(未设置)",
                      wayland ? wayland : "(未设置)");

        pangolin_render_->init(title, width, height);

        // 泵几帧渲染并处理事件，确保窗口首帧绘制、减少黑屏/无响应
        const int pump_total = 12;
        int pump_done = 0;
        UNICALIB_INFO("[Viz/Display] 首帧泵送: 开始 (共 {} 帧)", pump_total);
        for (int i = 0; i < pump_total; ++i) {
            bool ok = render_and_handle_input();
            if (!ok) {
                UNICALIB_WARN("[Viz/Display] 首帧泵送 第 {}/{} 帧 渲染返回 false，停止泵送 (可能窗口已关闭)", i + 1, pump_total);
                break;
            }
            pump_done++;
            if (i == 0 || i == pump_total - 1) {
                UNICALIB_INFO("[Viz/Display] 首帧泵送 第 {}/{} 帧 render_ok=true", i + 1, pump_total);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        UNICALIB_INFO("[Viz/Display] 首帧泵送完成 共 {} 帧 (窗口应已显示网格/坐标轴，尚未推送点云)；标定完成后请手动关闭窗口", pump_done);
    } else {
        if (!pangolin_render_)
            UNICALIB_DEBUG("[Viz/Display] start_display 跳过: Pangolin 渲染器未启用");
        else if (pangolin_render_->is_running())
            UNICALIB_DEBUG("[Viz/Display] start_display 跳过: 窗口已在运行");
    }
#else
    (void)title;
    (void)width;
    (void)height;
    UNICALIB_DEBUG("[Viz/Display] start_display 跳过: 未编译 UNICALIB_WITH_PANGOLIN");
#endif
}

void CalibVisualizer::close_display() {
#if UNICALIB_WITH_PANGOLIN
    if (pangolin_render_) {
        bool was_running = pangolin_render_->is_running();
        UNICALIB_INFO("[Viz/Display] 即将关闭可视化窗口 原因=用户关闭或流程结束 pangolin_running={}", was_running);
        if (was_running) {
            pangolin_render_->close();
            UNICALIB_INFO("[Viz/Display] 可视化窗口已关闭");
        }
    } else {
        UNICALIB_DEBUG("[Viz/Display] close_display 跳过: Pangolin 渲染器为空");
    }
#else
    UNICALIB_DEBUG("[Viz/Display] close_display 跳过: 未编译 UNICALIB_WITH_PANGOLIN");
#endif
}

void CalibVisualizer::spin() {
#if UNICALIB_WITH_PANGOLIN
    if (!pangolin_render_ || !pangolin_render_->is_running()) {
        UNICALIB_INFO("[Viz/Spin] spin 跳过: 窗口未创建或未运行");
        return;
    }
    UNICALIB_INFO("[Viz/Spin] 进入阻塞循环，等待用户手动关闭窗口（关闭窗口后程序将继续）");
    int frame_count = 0;
    const int log_interval = 60;  // 每 60 帧打一条
    while (!stopped_) {
        bool ok = render_and_handle_input();
        frame_count++;
        if (frame_count == 1 || frame_count % log_interval == 0) {
            UNICALIB_DEBUG("[Viz/Spin] 渲染帧 {}  render_ok={}", frame_count, ok);
        }
        if (!ok) {
            UNICALIB_INFO("[Viz/Spin] 窗口已关闭或收到退出请求，退出 spin 总帧数={}", frame_count);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    UNICALIB_INFO("[Viz/Spin] spin 结束");
#else
    UNICALIB_DEBUG("[Viz/Spin] spin 跳过: 未编译 UNICALIB_WITH_PANGOLIN");
#endif
}

void CalibVisualizer::spin_once(int ms) {
#if UNICALIB_WITH_PANGOLIN
    if (pangolin_render_ && pangolin_render_->is_running()) {
        render_and_handle_input();
    }
#else
    (void)ms;
#endif
    if (ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

void CalibVisualizer::generate_report(const std::string& output_path) {
    report_generator_->generate(output_path);
}

void CalibVisualizer::save_plots(const std::string& output_path) {
    if (config_.auto_save_plots && !output_path.empty() && cloud_viewer_) {
        std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        std::string filepath = output_path + "_" + timestamp + ".png";
        cloud_viewer_->save_screenshot(filepath);
        UNICALIB_INFO("Saved plot: {}", filepath);
    }
}

// ===========================================================================
// 2D 绘图工具
// ===========================================================================
cv::Mat CalibVisualizer::draw_optimization_convergence_curve(const std::vector<double>& iterations,
                                                              const std::vector<double>& costs,
                                                              const std::string& title,
                                                              int width, int height) {
    if (iterations.size() != costs.size() || iterations.empty()) {
        return cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    }

    const int W = width > 0 ? width : config_.plot_width;
    const int H = height > 0 ? height : config_.plot_height;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    // 绘制标题
    cv::putText(img, title, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 计算范围
    double min_cost = *std::min_element(costs.begin(), costs.end());
    double max_cost = *std::max_element(costs.begin(), costs.end());
    int max_iter = static_cast<int>(*std::max_element(iterations.begin(), iterations.end()));

    // 绘制坐标轴
    const int margin = 60;
    const int plot_w = W - 2 * margin;
    const int plot_h = H - 2 * margin;
    cv::line(img, cv::Point(margin, margin), cv::Point(margin, H - margin), cv::Scalar(0, 0, 0), 1);
    cv::line(img, cv::Point(margin, H - margin), cv::Point(W - margin, H - margin), cv::Scalar(0, 0, 0), 1);

    // 轴标签
    cv::putText(img, "Iteration", cv::Point(W / 2 - 40, H - margin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(img, "Cost", cv::Point(margin - 5, margin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 绘制曲线
    double cost_range = max_cost - min_cost;
    if (cost_range < 1e-10) cost_range = 1.0;

    for (size_t i = 1; i < iterations.size(); ++i) {
        int x1 = margin + static_cast<int>(iterations[i - 1] / max_iter * plot_w);
        int y1 = H - margin - static_cast<int>((costs[i - 1] - min_cost) / cost_range * plot_h);
        int x2 = margin + static_cast<int>(iterations[i] / max_iter * plot_w);
        int y2 = H - margin - static_cast<int>((costs[i] - min_cost) / cost_range * plot_h);
        cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(60, 160, 60), 2);
        cv::circle(img, cv::Point(x2, y2), 4, cv::Scalar(200, 60, 60), -1);
    }

    return img;
}

cv::Mat CalibVisualizer::draw_camera_reprojection_errors(
    const std::vector<Eigen::Vector2d>& corners_2d,
    const std::vector<Eigen::Vector2d>& corners_reprojected,
    const std::string& title) {
    const int W = config_.plot_width;
    const int H = config_.plot_height;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
    if (corners_2d.empty() && corners_reprojected.empty()) {
        cv::putText(img, title, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        return img;
    }
    for (const auto& p : corners_2d)
        cv::circle(img, cv::Point(static_cast<int>(p.x()), static_cast<int>(p.y())), 3, cv::Scalar(0, 0, 255), -1);
    for (const auto& p : corners_reprojected)
        cv::circle(img, cv::Point(static_cast<int>(p.x()), static_cast<int>(p.y())), 2, cv::Scalar(0, 255, 0), -1);
    cv::putText(img, title, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    return img;
}

cv::Mat CalibVisualizer::plot_time_offset_convergence_curve(const std::vector<double>& time_offsets,
                                                             const std::vector<double>& costs,
                                                             const std::string& title) {
    if (time_offsets.empty() || costs.empty()) return cv::Mat();

    std::vector<double> sorted_offsets = time_offsets;
    std::sort(sorted_offsets.begin(), sorted_offsets.end());
    double min_time_offset = sorted_offsets.front();
    double max_time_offset = sorted_offsets.back();

    const int W = config_.plot_width;
    const int H = config_.plot_height;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    const int margin = 50;
    const int plot_w = W - 2 * margin;
    const int plot_h = H - 2 * margin;

    // 绘制坐标轴
    cv::line(img, cv::Point(margin, margin), cv::Point(margin, H - margin), cv::Scalar(0, 0, 0), 1);
    cv::line(img, cv::Point(margin, H - margin), cv::Point(W - margin, H - margin), cv::Scalar(0, 0, 0), 1);

    // 轴标签
    cv::putText(img, "Time Offset (s)", cv::Point(W / 2 - 50, H - margin + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(img, "Cost", cv::Point(margin - 5, margin + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 绘制数据点
    if (time_offsets.size() != costs.size()) return img;

    double time_range = max_time_offset - min_time_offset;
    double max_cost = *std::max_element(costs.begin(), costs.end());
    double min_cost = *std::min_element(costs.begin(), costs.end());
    double cost_range = max_cost - min_cost;
    if (cost_range < 1e-10) cost_range = 1.0;

    for (size_t i = 0; i < time_offsets.size(); ++i) {
        int x = margin + static_cast<int>((time_offsets[i] - min_time_offset) / time_range * plot_w);
        int y = H - margin - static_cast<int>((costs[i] - min_cost) / cost_range * plot_h);
        cv::circle(img, cv::Point(x, y), 3, cv::Scalar(200, 50, 50), -1);
    }

    return img;
}

cv::Mat CalibVisualizer::draw_imu_lidar_result_summary(const ExtrinsicSE3& coarse,
                                                       const ExtrinsicSE3& fine,
                                                       const std::vector<double>& rot_errors,
                                                       const std::vector<double>& trans_errors) {
    const int W = 800, H = 400;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    // 绘制标题
    cv::putText(img, "IMU-LiDAR Calibration Results", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 计算变化量
    Eigen::Vector3d trans_diff = fine.POS_TargetInRef - coarse.POS_TargetInRef;
    Eigen::Vector3d rpy_diff = fine.euler_deg() - coarse.euler_deg();

    // 绘制结果
    cv::putText(img, "Coarse -> Fine Changes:", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(img, cv::format("  Translation: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f] m",
                  trans_diff.x(), trans_diff.y(), trans_diff.z(),
                  fine.POS_TargetInRef.x(), fine.POS_TargetInRef.y(), fine.POS_TargetInRef.z()),
                cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(img, cv::format("  Rotation: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f] deg",
                  rpy_diff.x(), rpy_diff.y(), rpy_diff.z(),
                  fine.euler_deg().x(), fine.euler_deg().y(), fine.euler_deg().z()),
                cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(img, cv::format("  Time Offset: %.4f -> %.4f ms",
                  coarse.time_offset_s * 1000, fine.time_offset_s * 1000),
                cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 绘制误差统计
    if (!rot_errors.empty()) {
        cv::putText(img, "Rotation Error Distribution", cv::Point(10, 200), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        // 绘制误差柱状图
        double max_err = *std::max_element(rot_errors.begin(), rot_errors.end());
        int margin = 60;
        int bar_width = 680;
        int bar_height = 150;
        for (size_t i = 0; i < rot_errors.size(); ++i) {
            double err_normalized = rot_errors[i] / max_err;
            int x = margin + static_cast<int>(i * bar_width / rot_errors.size());
            int bar_h = static_cast<int>(err_normalized * bar_height);
            int color = err_normalized < 0.3 ? static_cast<int>((1.0 - err_normalized) * 255) : static_cast<int>((1.0 - err_normalized) * 255);
            cv::rectangle(img, cv::Point(x, H - margin - bar_h - 10),
                          cv::Point(x + bar_width - 5, H - margin),
                          cv::Scalar(color, color, 0), cv::FILLED);
        }
    }

    if (!trans_errors.empty()) {
        cv::putText(img, "Translation Error Distribution", cv::Point(10, 340), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        // 绘制误差柱状图
        double max_err = *std::max_element(trans_errors.begin(), trans_errors.end());
        int margin = 60;
        int bar_width = 680;
        int bar_height = 150;
        for (size_t i = 0; i < trans_errors.size(); ++i) {
            double err_normalized = trans_errors[i] / max_err;
            int x = margin + static_cast<int>(i * bar_width / trans_errors.size());
            int bar_h = static_cast<int>(err_normalized * bar_height);
            int color = err_normalized < 0.3 ? static_cast<int>((1.0 - err_normalized) * 255) : static_cast<int>((1.0 - err_normalized) * 255);
            cv::rectangle(img, cv::Point(x, H - margin - bar_h - 10),
                          cv::Point(x + bar_width - 5, H - margin),
                          cv::Scalar(color, color, 0), cv::FILLED);
        }
    }

    return img;
}

// ===========================================================================
// 添加可视化数据
// ===========================================================================
void CalibVisualizer::add_visualization_data(const std::string& key, const VisualizationData& data) {
    std::lock_guard<std::mutex> lock(mtx_);
    viz_data_[key] = data;
}

// ===========================================================================
// 交互式手动调整实现
// ===========================================================================

namespace {

// 便捷旋转函数
Eigen::Matrix3d euler_to_rotation(double roll_deg, double pitch_deg, double yaw_deg) {
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;
    
    Eigen::AngleAxisd roll_axis(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_axis(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_axis(yaw, Eigen::Vector3d::UnitZ());
    
    return (yaw_axis * pitch_axis * roll_axis).matrix();
}

}  // anonymous namespace

std::optional<ExtrinsicSE3> CalibVisualizer::run_interactive_adjustment(
    const ExtrinsicSE3& init_extrinsic,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& /*ref_cloud*/,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& /*target_cloud*/,
    const cv::Mat& image,
    const CameraIntrinsics& /*cam_intrin*/) {
    
    // OpenCV交互模式
    UNICALIB_INFO("[Visualizer] 启动OpenCV交互模式...");
    
    cv::namedWindow(config_.window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(config_.window_title, config_.window_width, config_.window_height);
    
    auto current_extrinsic = init_extrinsic;
    bool running = true;
    bool accepted = false;
    
    cv::Mat display;
    if (!image.empty()) {
        display = image.clone();
    } else {
        display = cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 50, 50));
    }
    
    while (running) {
        cv::Mat show = display.clone();
        
        // 显示当前外参
        std::string info = "T: (" + 
            std::to_string(current_extrinsic.POS_TargetInRef[0]) + ", " +
            std::to_string(current_extrinsic.POS_TargetInRef[1]) + ", " +
            std::to_string(current_extrinsic.POS_TargetInRef[2]) + ")";
        cv::putText(show, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        cv::putText(show, "Controls: WASD=Translate, IJKL=Rotate, R=Reset, Enter=Accept, Esc=Cancel", 
            cv::Point(10, show.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        cv::imshow(config_.window_title, show);
        
        int key = cv::waitKey(30);
        if (key >= 0) {
            char k = static_cast<char>(key);
            
            // 平移
            if (k == 'w') current_extrinsic.POS_TargetInRef += Eigen::Vector3d(0, 0, config_.trans_step_m);
            if (k == 's') current_extrinsic.POS_TargetInRef -= Eigen::Vector3d(0, 0, config_.trans_step_m);
            if (k == 'a') current_extrinsic.POS_TargetInRef -= Eigen::Vector3d(config_.trans_step_m, 0, 0);
            if (k == 'd') current_extrinsic.POS_TargetInRef += Eigen::Vector3d(config_.trans_step_m, 0, 0);
            if (k == 'q') current_extrinsic.POS_TargetInRef += Eigen::Vector3d(0, config_.trans_step_m, 0);
            if (k == 'e') current_extrinsic.POS_TargetInRef -= Eigen::Vector3d(0, config_.trans_step_m, 0);
            
            // 旋转
            if (k == 'i') {
                Eigen::Matrix3d R = euler_to_rotation(config_.rot_step_deg, 0, 0);
                current_extrinsic.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic.SO3_TargetInRef.matrix());
            }
            if (k == 'k') {
                Eigen::Matrix3d R = euler_to_rotation(-config_.rot_step_deg, 0, 0);
                current_extrinsic.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic.SO3_TargetInRef.matrix());
            }
            if (k == 'j') {
                Eigen::Matrix3d R = euler_to_rotation(0, config_.rot_step_deg, 0);
                current_extrinsic.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic.SO3_TargetInRef.matrix());
            }
            if (k == 'l') {
                Eigen::Matrix3d R = euler_to_rotation(0, -config_.rot_step_deg, 0);
                current_extrinsic.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic.SO3_TargetInRef.matrix());
            }
            if (k == 'u') {
                Eigen::Matrix3d R = euler_to_rotation(0, 0, config_.rot_step_deg);
                current_extrinsic.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic.SO3_TargetInRef.matrix());
            }
            if (k == 'o') {
                Eigen::Matrix3d R = euler_to_rotation(0, 0, -config_.rot_step_deg);
                current_extrinsic.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic.SO3_TargetInRef.matrix());
            }
            
            // 重置/确认/取消
            if (k == 'r') current_extrinsic = init_extrinsic;
            if (key == 13) { accepted = true; running = false; }  // Enter
            if (key == 27) { running = false; }  // Esc
            
            // 步长调整
            if (k == '+' || k == '=') {
                config_.trans_step_m = std::min(config_.trans_step_m * 1.2, 0.1);
                config_.rot_step_deg = std::min(config_.rot_step_deg * 1.2, 10.0);
            }
            if (k == '-') {
                config_.trans_step_m = std::max(config_.trans_step_m / 1.2, 0.001);
                config_.rot_step_deg = std::max(config_.rot_step_deg / 1.2, 0.1);
            }
        }
    }
    
    cv::destroyWindow(config_.window_title);
    
    if (accepted) return current_extrinsic;
    return std::nullopt;
}

void CalibVisualizer::set_on_extrinsic_changed(ExtrinsicChangedCallback cb) {
    extrinsic_changed_cb_ = std::move(cb);
    UNICALIB_DEBUG("[Visualizer] set_on_extrinsic_changed");
}

void CalibVisualizer::set_on_accept(AcceptCallback cb) {
    accept_cb_ = std::move(cb);
    UNICALIB_DEBUG("[Visualizer] set_on_accept");
}

void CalibVisualizer::set_on_cancel(CancelCallback cb) {
    cancel_cb_ = std::move(cb);
    UNICALIB_DEBUG("[Visualizer] set_on_cancel");
}

void CalibVisualizer::set_rotation_mode(VizConfig::RotationEditMode mode) {
    config_.rotation_mode = mode;
    UNICALIB_INFO("[Visualizer] 旋转模式: {}", static_cast<int>(mode));
}

void CalibVisualizer::translate(double dx, double dy, double dz) {
    // 保存历史状态
    if (history_index_ < static_cast<int>(history_.size()) - 1) {
        history_.resize(history_index_ + 1);
    }
    if (static_cast<int>(history_.size()) >= MAX_HISTORY) {
        history_.erase(history_.begin());
    }
    history_.push_back(current_extrinsic_);
    history_index_ = history_.size() - 1;

    // 应用平移
    current_extrinsic_.POS_TargetInRef += Eigen::Vector3d(dx, dy, dz);

    // 更新实时数据
    if (target_cloud_ && ref_cloud_) {
        update_realtime_data(ref_cloud_, target_cloud_, current_extrinsic_);
    }

    UNICALIB_DEBUG("[Visualizer] 平移: ({}, {}, {})", dx, dy, dz);
}

void CalibVisualizer::rotate(double rx, double ry, double rz) {
    // 保存历史状态
    if (history_index_ < static_cast<int>(history_.size()) - 1) {
        history_.resize(history_index_ + 1);
    }
    if (static_cast<int>(history_.size()) >= MAX_HISTORY) {
        history_.erase(history_.begin());
    }
    history_.push_back(current_extrinsic_);
    history_index_ = history_.size() - 1;

    // 根据旋转模式应用旋转
    double rad = config_.rot_step_deg * M_PI / 180.0;
    Eigen::Matrix3d R;

    if (config_.rotation_mode == VizConfig::RotationEditMode::AXIS_ANGLE) {
        // 轴角模式: rx/ry/rz 表示绕各轴的旋转角度
        Eigen::AngleAxisd rx_axis(rx * rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry_axis(ry * rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz_axis(rz * rad, Eigen::Vector3d::UnitZ());
        R = (rz_axis * ry_axis * rx_axis).matrix();
    } else if (config_.rotation_mode == VizConfig::RotationEditMode::EULER) {
        // 欧拉角模式
        R = euler_to_rotation(rx, ry, rz);
    } else if (config_.rotation_mode == VizConfig::RotationEditMode::ROTATION_VECTOR) {
        // 旋转向量模式
        Eigen::Vector3d vec(rx, ry, rz);
        double angle = vec.norm() * rad;
        if (angle > 1e-10) {
            vec.normalize();
            R = Eigen::AngleAxisd(angle, vec).matrix();
        } else {
            R = Eigen::Matrix3d::Identity();
        }
    } else {
        // 四元数或其他: 暂时用欧拉角
        R = euler_to_rotation(rx, ry, rz);
    }

    // 应用旋转
    current_extrinsic_.SO3_TargetInRef = Sophus::SO3d(R * current_extrinsic_.SO3_TargetInRef.matrix());

    // 更新实时数据
    if (target_cloud_ && ref_cloud_) {
        update_realtime_data(ref_cloud_, target_cloud_, current_extrinsic_);
    }

    UNICALIB_DEBUG("[Visualizer] 旋转 (模式 {}): rx={}, ry={}, rz={}",
                   static_cast<int>(config_.rotation_mode), rx, ry, rz);
}

VizConfig::RotationEditMode CalibVisualizer::get_rotation_mode() const {
    return config_.rotation_mode;
}

bool CalibVisualizer::undo() {
    if (history_index_ > 0) {
        history_index_--;
        current_extrinsic_ = history_[history_index_];
        if (target_cloud_ && ref_cloud_) {
            update_realtime_data(ref_cloud_, target_cloud_, current_extrinsic_);
        }
        UNICALIB_DEBUG("[Visualizer] undo: {}", history_index_);
        return true;
    }
    return false;
}

bool CalibVisualizer::redo() {
    if (history_index_ < static_cast<int>(history_.size()) - 1) {
        history_index_++;
        current_extrinsic_ = history_[history_index_];
        if (target_cloud_ && ref_cloud_) {
            update_realtime_data(ref_cloud_, target_cloud_, current_extrinsic_);
        }
        UNICALIB_DEBUG("[Visualizer] redo: {}", history_index_);
        return true;
    }
    return false;
}

void CalibVisualizer::reset_to_initial() {
    if (!history_.empty()) {
        current_extrinsic_ = history_.front();
        history_index_ = 0;
        if (target_cloud_ && ref_cloud_) {
            update_realtime_data(ref_cloud_, target_cloud_, current_extrinsic_);
        }
        UNICALIB_INFO("[Visualizer] 重置到初始外参");
    }
}

std::optional<ExtrinsicSE3> CalibVisualizer::get_current_extrinsic() const {
    return current_extrinsic_;
}

bool CalibVisualizer::start_video_recording(const std::string& output_path, int fps) {
    UNICALIB_INFO("[Visualizer] 开始视频录制: {}", output_path);
    return false;
}

void CalibVisualizer::stop_video_recording() {
    UNICALIB_INFO("[Visualizer] 停止视频录制");
}

void CalibVisualizer::save_frame_to_video() {
    UNICALIB_DEBUG("[Visualizer] 保存帧到视频");
}

void CalibVisualizer::update_realtime_data(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& ref_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud,
    const ExtrinsicSE3& extrinsic) {

    // 缓存实时数据
    ref_cloud_ = ref_cloud;
    target_cloud_ = target_cloud;
    current_extrinsic_ = extrinsic;

    // 更新 Pangolin 渲染器
#if UNICALIB_WITH_PANGOLIN
    if (!pangolin_render_) {
        UNICALIB_WARN("[Viz/Display] update_realtime_data 跳过: pangolin_render_ 为空");
    } else if (!pangolin_render_->is_running()) {
        UNICALIB_WARN("[Viz/Display] update_realtime_data 跳过: 窗口未运行或已请求退出");
    } else {
        pangolin_render_->set_ref_cloud(ref_cloud, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        pangolin_render_->set_target_cloud(target_cloud, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = extrinsic.SO3_TargetInRef.matrix();
        T.block<3, 1>(0, 3) = extrinsic.POS_TargetInRef;
        pangolin_render_->set_extrinsic(T);
        size_t ref_n = ref_cloud ? ref_cloud->size() : 0;
        size_t tgt_n = target_cloud ? target_cloud->size() : 0;
        UNICALIB_INFO("[Viz/Display] 已向 3D 窗口推送点云 ref={} 点 target={} 点 外参已设置 (下一帧渲染将显示)", ref_n, tgt_n);
    }
#endif

    // 触发回调
    if (extrinsic_changed_cb_) {
        extrinsic_changed_cb_(extrinsic);
    }

    UNICALIB_DEBUG("[Visualizer] 实时数据已更新: ref={}, target={}",
                   ref_cloud ? ref_cloud->size() : 0,
                   target_cloud ? target_cloud->size() : 0);
}

bool CalibVisualizer::render_and_handle_input() {
#if UNICALIB_WITH_PANGOLIN
    static int s_render_call_count = 0;
    const bool log_this_call = (s_render_call_count < 10);
    if (log_this_call) s_render_call_count++;

    if (!pangolin_render_) {
        UNICALIB_WARN("[Viz/Render] render_and_handle_input 返回 false: pangolin_render_ 为空");
        return false;
    }
    if (!pangolin_render_->is_running()) {
        UNICALIB_WARN("[Viz/Render] render_and_handle_input 返回 false: 窗口未运行或已请求退出");
        return false;
    }

    // 渲染 Pangolin 场景（异常时视为渲染失败，避免崩溃）
    bool continue_render = false;
    try {
        if (log_this_call) {
            UNICALIB_INFO("[Viz/Render] render_and_handle_input 第 {} 次调用 即将调用 pangolin_render_->render()", s_render_call_count);
        }
        continue_render = pangolin_render_->render();
        if (log_this_call) {
            UNICALIB_INFO("[Viz/Render] render_and_handle_input 第 {} 次 render() 返回 continue_render={}", s_render_call_count, continue_render);
        }
    } catch (const std::exception& e) {
        UNICALIB_WARN("[Viz/Render] render 抛出异常: {} 视为窗口不可用", e.what());
    } catch (...) {
        UNICALIB_WARN("[Viz/Render] render 抛出未知异常 视为窗口不可用");
    }
    if (!continue_render && logger_) {
        UNICALIB_INFO("[Viz/Render] render 返回 false (窗口可能已关闭或 ShouldQuit)");
    }
#else
    // Pangolin 未启用，返回 false 表示没有进行渲染
    (void)config_;  // 消除未使用警告
    return false;
#endif

#if UNICALIB_WITH_PANGOLIN
    // 处理键盘输入 (轴角模式)
    if (config_.enable_keyboard) {
        // 注意: Pangolin 3D handler 会自动处理鼠标交互
        // 键盘控制需要在外部调用 translate/rotate 方法
    }

    return continue_render;
#else
    return false;
#endif
}

}  // namespace ns_unicalib
