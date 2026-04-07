/**
 * UniCalib — 相机内参标定应用
 * 使用: unicalib_camera_intrinsic --config <config.yaml>
 *       unicalib_camera_intrinsic --images_dir <dir> --model pinhole|fisheye
 *
 * 数据源（与 IMU 内参一致）:
 *   - 图像目录: data.camera.<id>.images_dir（相对路径相对于 --data-dir）
 *   - ROS2 bag: ros2.use_ros2_bag + ros2.ros2_bag_file + 相机话题
 *   - 实时标定: ros2.use_ros2_topics: true，订阅相机话题采集帧
 *
 * 输出:
 *   - 标定结果 YAML
 *   - 每帧检测结果图像
 *   - 重投影误差分布图
 *   - HTML 报告
 */

#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/accuracy_logger.h"
#include "unicalib/intrinsic/camera_calib.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <thread>
#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
#include "unicalib/io/ros2_data_source.h"
#endif

namespace fs = std::filesystem;
using namespace ns_unicalib;

// 先验内参 (无棋盘格且 DM-Calib 不可用时): cx=w/2, cy=h/2, fx=fy≈0.8*min(w,h)
static CameraIntrinsics make_prior_intrinsic(int width, int height) {
    CameraIntrinsics out;
    out.model = CameraIntrinsics::Model::PINHOLE;
    out.width = width;
    out.height = height;
    out.cx = width * 0.5;
    out.cy = height * 0.5;
    double f = std::min(width, height) * 0.8;
    out.fx = out.fy = f;
    out.dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
    out.rms_reproj_error = -1.0;  // N/A
    out.num_images_used = 0;
    return out;
}

// 解析数据路径：相对路径与 base 拼接；绝对路径原样返回
static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    if (base.empty()) return path;
    fs::path p(path);
    if (p.is_absolute()) return path;
    return (fs::path(base) / p).lexically_normal().string();
}

// ===================================================================
// 收集图像路径
// ===================================================================
std::vector<std::string> collect_images(const std::string& dir) {
    std::vector<std::string> paths;
    const std::vector<std::string> exts = {".png", ".jpg", ".jpeg", ".bmp", ".tiff"};
    for (const auto& entry : fs::directory_iterator(dir)) {
        std::string ext = entry.path().extension().string();
        for (auto& c : ext) c = std::tolower(static_cast<unsigned char>(c));
        if (std::find(exts.begin(), exts.end(), ext) != exts.end()) {
            paths.push_back(entry.path().string());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

// ===================================================================
// 单相机标定任务（配置中每个 type:camera 对应一项）
// ===================================================================
struct CameraTask {
    std::string sensor_id;
    std::string camera_topic;
    std::string images_dir;
    std::string reference_intrinsic_yaml;
};

// ===================================================================
// 全局标定配置（多相机共用）
// ===================================================================
struct GlobalCamIntrinConfig {
    std::string output_dir;
    std::string data_dir;
    std::string results_camera_intrinsic;
    std::string model_str;
    bool prefer_no_checkerboard = true;
    CameraIntrinsicCalibrator::Config cfg;
    std::string dm_calib_repo;
    std::string dm_calib_model;
    bool dm_calib_use_cpu = true;
    int dm_calib_timeout_sec = 600;
    int dm_calib_denoise_steps = 0;
    int dm_calib_ensemble_size = 0;
    int dm_calib_processing_res = 0;
    int dm_calib_max_images = 25;
};

// ===================================================================
// 单相机内参标定（供多相机循环调用）
// ===================================================================
static int run_single_camera(
    const CameraTask& task,
    const GlobalCamIntrinConfig& g,
    std::vector<std::string> image_paths,
    std::vector<cv::Mat> image_mats,
    int image_width,
    int image_height)
{
    const std::string& sensor_id = task.sensor_id;
    const std::string& model_str = g.model_str;
    const std::string& reference_intrinsic_yaml = task.reference_intrinsic_yaml;

    UNICALIB_INFO("========== 相机内参标定: {} ==========", sensor_id);

    if (image_width <= 0 || image_height <= 0) {
        if (!image_paths.empty()) {
            for (size_t k = 0; k < std::min(image_paths.size(), size_t(5)); ++k) {
                cv::Mat img = cv::imread(image_paths[k]);
                if (!img.empty()) {
                    image_width = img.cols;
                    image_height = img.rows;
                    UNICALIB_INFO("图像分辨率 (从第 {} 张读取): {}×{}", k + 1, image_width, image_height);
                    break;
                }
            }
        }
        if (image_width <= 0 || image_height <= 0) {
            UNICALIB_ERROR("无法获取图像尺寸: {}", sensor_id);
            return 1;
        }
    }

    std::optional<CameraIntrinsics> fallback_intrin;
    std::optional<CameraIntrinsics> result;
    std::string result_method = "prior";

    if (g.prefer_no_checkerboard && image_width > 0 && image_height > 0) {
        std::vector<std::string> paths_for_dm;
        const size_t max_for_dm = static_cast<size_t>(std::max(1, g.dm_calib_max_images));
        std::string temp_dm_dir = g.output_dir + "/dm_calib_input/" + sensor_id;
        fs::create_directories(temp_dm_dir);

        if (!image_paths.empty()) {
            for (size_t i = 0; i < std::min(max_for_dm, image_paths.size()); ++i)
                paths_for_dm.push_back(image_paths[i]);
            UNICALIB_INFO("[{}] 步骤1/2: 无棋盘格标定 — 使用前 {} 张图像尝试 DM-Calib", sensor_id, paths_for_dm.size());
        } else if (!image_mats.empty()) {
            for (size_t i = 0; i < std::min(max_for_dm, image_mats.size()); ++i) {
                std::string p = temp_dm_dir + "/frame_" + std::to_string(i) + ".png";
                if (cv::imwrite(p, image_mats[i]))
                    paths_for_dm.push_back(p);
            }
            UNICALIB_INFO("[{}] 步骤1/2: 无棋盘格标定 — 已写入 {} 张图像到 {} 供 DM-Calib", sensor_id, paths_for_dm.size(), temp_dm_dir);
        }

        if (!paths_for_dm.empty()) {
            ns_unicalib::AICoarseCalibManager::Config ai_cfg;
            ai_cfg.work_dir = g.output_dir + "/ai_work/" + sensor_id;
            ai_cfg.dm_calib.repo_dir = g.dm_calib_repo;
            ai_cfg.dm_calib.work_dir = ai_cfg.work_dir + "/dmcalib";
            ai_cfg.dm_calib.model_path = g.dm_calib_model.empty() ? g.dm_calib_repo + "/model" : g.dm_calib_model;
            ai_cfg.dm_calib.use_cpu = g.dm_calib_use_cpu;
            ai_cfg.dm_calib.timeout_sec = g.dm_calib_timeout_sec;
            if (g.dm_calib_denoise_steps > 0)   ai_cfg.dm_calib.denoise_steps   = g.dm_calib_denoise_steps;
            if (g.dm_calib_ensemble_size > 0)   ai_cfg.dm_calib.ensemble_size  = g.dm_calib_ensemble_size;
            if (g.dm_calib_processing_res > 0)  ai_cfg.dm_calib.processing_res  = g.dm_calib_processing_res;

            ns_unicalib::AICoarseCalibManager ai_mgr(ai_cfg);
            if (ai_mgr.check_dm_calib()) {
                auto no_cb = ai_mgr.coarse_cam_intrinsic(paths_for_dm, image_width, image_height, sensor_id);
                if (no_cb.has_value()) {
                    fallback_intrin = *no_cb;
                    result_method = "dm_calib";
                    UNICALIB_INFO("[{}] 无棋盘格标定成功 (DM-Calib): fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                                  sensor_id, fallback_intrin->fx, fallback_intrin->fy, fallback_intrin->cx, fallback_intrin->cy);
                } else {
                    fallback_intrin = make_prior_intrinsic(image_width, image_height);
                    UNICALIB_WARN("[{}] DM-Calib 推理失败或置信度不足，已回退先验内参", sensor_id);
                }
            } else {
                fallback_intrin = make_prior_intrinsic(image_width, image_height);
                UNICALIB_WARN("[{}] DM-Calib 不可用，已回退先验内参", sensor_id);
            }
        } else {
            fallback_intrin = make_prior_intrinsic(image_width, image_height);
        }
    }

    UNICALIB_INFO("[{}] 步骤2/2: 棋盘格角点检测与精化...", sensor_id);
    auto t_calib_start = std::chrono::high_resolution_clock::now();
    CameraIntrinsicCalibrator::Config calib_cfg = g.cfg;
    calib_cfg.checkerboard_optional = g.prefer_no_checkerboard;
    CameraIntrinsicCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([&sensor_id](int cur, int total) {
        if (cur % 10 == 0 || cur == total - 1)
            std::cout << "\r  [" << sensor_id << "] 检测角点: " << cur << "/" << total << "  " << std::flush;
    });

    if (!image_mats.empty()) {
        result = calibrator.calibrate(image_mats, image_width, image_height);
    } else {
        result = calibrator.calibrate(image_paths);
    }
    std::cout << std::endl;
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t_calib_start).count();

    if (result.has_value()) {
        result_method = "opencv_chessboard";
        UNICALIB_INFO("[{}] 棋盘格有效帧充足，采用棋盘格标定结果 (RMS={:.4f} px)", sensor_id, result->rms_reproj_error);
    } else {
        if (g.prefer_no_checkerboard && fallback_intrin.has_value()) {
            result = fallback_intrin;
            UNICALIB_WARN("[{}] 棋盘格有效帧不足，采用无棋盘格结果 (method={})", sensor_id, result_method);
        } else {
            append_calib_accuracy(g.output_dir, CalibAccuracyTask::CAM_INTRINSIC, false, -1.0, elapsed_ms, {});
            UNICALIB_ERROR("[{}] Camera intrinsic calibration FAILED", sensor_id);
            return 1;
        }
    }

    const auto& intrin = result.value();
    std::map<std::string, std::string> extra;
    extra["num_images"] = std::to_string(intrin.num_images_used);
    extra["method"] = result_method;
    if (!reference_intrinsic_yaml.empty() && fs::exists(reference_intrinsic_yaml)) {
        try {
            YAML::Node ny = YAML::LoadFile(reference_intrinsic_yaml);
            double ref_fx = 0, ref_fy = 0, ref_cx = 0, ref_cy = 0;
            if (ny["projection_parameters"]) {
                const auto& pp = ny["projection_parameters"];
                if (pp["fx"]) ref_fx = pp["fx"].as<double>();
                if (pp["fy"]) ref_fy = pp["fy"].as<double>();
                if (pp["cx"]) ref_cx = pp["cx"].as<double>();
                if (pp["cy"]) ref_cy = pp["cy"].as<double>();
            }
            double dfx = intrin.fx - ref_fx, dfy = intrin.fy - ref_fy;
            double dcx = intrin.cx - ref_cx, dcy = intrin.cy - ref_cy;
            UNICALIB_INFO("[{}] 与参考标定对比: Δfx={:.2f} Δfy={:.2f} Δcx={:.2f} Δcy={:.2f}", sensor_id, dfx, dfy, dcx, dcy);
            extra["ref_fx"] = std::to_string(ref_fx);
            extra["ref_fy"] = std::to_string(ref_fy);
            extra["diff_fx"] = std::to_string(dfx);
            extra["diff_fy"] = std::to_string(dfy);
            extra["diff_cx"] = std::to_string(dcx);
            extra["diff_cy"] = std::to_string(dcy);
        } catch (const std::exception& e) {
            UNICALIB_WARN("[{}] 读取参考标定文件失败: {}", sensor_id, e.what());
        }
    }

    // 畸变系数：OpenCV 顺序 k1, k2, p1, p2 [, k3 ...]；无棋盘格/DM-Calib 时多为 0
    double k1 = intrin.dist_coeffs.size() > 0 ? intrin.dist_coeffs[0] : 0.0;
    double k2 = intrin.dist_coeffs.size() > 1 ? intrin.dist_coeffs[1] : 0.0;
    double p1 = intrin.dist_coeffs.size() > 2 ? intrin.dist_coeffs[2] : 0.0;
    double p2 = intrin.dist_coeffs.size() > 3 ? intrin.dist_coeffs[3] : 0.0;

    // 与 results/camera_intrinsic/*.yaml 格式一致，便于后续标定读取
    YAML::Node out;
    out["sensor_id"] = sensor_id;
    out["model"]     = model_str;
    out["method"]    = result_method;
    out["width"]     = intrin.width;
    out["height"]    = intrin.height;
    out["distortion_model"] = "radial-tangential";
    out["distortion_parameters"]["k1"] = k1;
    out["distortion_parameters"]["k2"] = k2;
    out["distortion_parameters"]["p1"] = p1;
    out["distortion_parameters"]["p2"] = p2;
    out["projection_parameters"]["fx"] = intrin.fx;
    out["projection_parameters"]["fy"] = intrin.fy;
    out["projection_parameters"]["cx"] = intrin.cx;
    out["projection_parameters"]["cy"] = intrin.cy;
    out["rms_reproj_error"] = intrin.rms_reproj_error;
    out["num_images"] = intrin.num_images_used;

    std::string result_subdir = g.output_dir;
    if (!g.results_camera_intrinsic.empty()) {
        result_subdir = g.output_dir + "/" + g.results_camera_intrinsic;
        fs::create_directories(result_subdir);
    }
    std::string yaml_path = result_subdir + "/camera_intrinsic_" + sensor_id + ".yaml";
    {
        std::ofstream f(yaml_path);
        if (!f.is_open()) {
            UNICALIB_ERROR("无法写入结果文件: {}", yaml_path);
            return 1;
        }
        f << out;
        UNICALIB_INFO("Saved: {}", yaml_path);
    }

    append_calib_accuracy(g.output_dir, CalibAccuracyTask::CAM_INTRINSIC,
                         true, intrin.rms_reproj_error, elapsed_ms, extra);

    std::cout << "\n┌────────────────────────────────────────────────┐\n";
    std::cout << "│ 相机内参 " << std::setw(24) << sensor_id << " │\n";
    std::cout << "├──────────────────────┬──────────────────────────┤\n";
    std::cout << "│ 来源/方法            │ " << std::setw(24) << result_method << " │\n";
    std::cout << "│ 分辨率               │ " << std::setw(24)
              << (std::to_string(intrin.width) + "×" + std::to_string(intrin.height)) << " │\n";
    std::cout << "│ 有效图像帧           │ " << std::setw(24) << intrin.num_images_used << " │\n";
    std::cout << "│ fx / fy / cx / cy    │ " << std::setw(24) << std::fixed
              << std::setprecision(1) << intrin.fx << " " << intrin.fy << " " << intrin.cx << " " << intrin.cy << " │\n";
    std::cout << "└──────────────────────┴──────────────────────────┘\n";
    std::cout << "结果: " << yaml_path << "\n\n";

    return 0;
}

// ===================================================================
// 主函数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << R"(
 ╔══════════════════════════════════════════════╗
 ║    UniCalib — 相机内参标定 (针孔 + 鱼眼)    ║
 ╚══════════════════════════════════════════════╝
)" << std::endl;

    int main_exit = 0;  // 供 UNICALIB_MAIN_TRY_END 使用（宏展开后 return 在 try 外，需在 try 外可见）
    UNICALIB_MAIN_TRY_BEGIN

    // --- 参数解析 ---
    std::string images_dir;
    std::string data_dir;
    std::string model_str = "pinhole";
    std::string output_dir = "./results";
    std::string sensor_id = "cam_0";
    bool use_ros2_bag = false;
    bool use_ros2_realtime = false;
    std::string ros2_bag_file;
    std::string camera_topic;
    double realtime_timeout_sec = 60.0;
    size_t realtime_max_frames = 0;  // 0 = 按超时采集
    bool prefer_no_checkerboard = true;  // 优先无棋盘格标定，避免无棋盘格时失败
    std::string dm_calib_repo;           // third_party.dm_calib 或 env UNICALIB_DM_CALIB
    std::string dm_calib_model;         // third_party.dm_calib_model 或 env UNICALIB_DM_CALIB_MODEL，空则用 repo/model
    bool dm_calib_use_cpu = true;       // third_party.dm_calib_use_cpu：true=CPU 推理（GPU 不兼容时），false=用 GPU
    int dm_calib_timeout_sec = 600;    // third_party.dm_calib_timeout_sec：单次推理超时(秒)，CPU 推理建议 600+
    int dm_calib_denoise_steps = 0;    // >0 时传 Python，CPU 加速可设 10～12
    int dm_calib_ensemble_size = 0;   // >0 时传 Python，CPU 加速可设 1
    int dm_calib_processing_res = 0;   // >0 时传 Python，CPU 加速可设 512
    int dm_calib_max_images = 25;      // 参与 DM-Calib 推理的最大图像数；精度优先建议 20～30，取中位数更稳
    std::string config_ai_root;         // 配置中的 ai_root，用于回退 DM-Calib 路径
    std::string results_camera_intrinsic; // results.camera_intrinsic 子目录名；非空时结果写入 output_dir/<该子目录>/camera_intrinsic_<id>.yaml

    std::vector<CameraTask> camera_tasks;  // 多相机：配置中所有 type:camera 对应一项；单相机时仅一项

    CameraIntrinsicCalibrator::Config cfg;
    cfg.target.type = TargetConfig::Type::CHESSBOARD;
    cfg.target.cols = 9;
    cfg.target.rows = 6;
    cfg.target.square_size_m = 0.025;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            YAML::Node n = YAML::LoadFile(argv[++i]);
            if (n["data"] && n["sensors"]) {
                // 收集所有 type:camera 的传感器，不再只取第一个
                std::map<std::string, std::string> ros2_camera_topics;  // id -> topic（来自 ros2.camera_topics）
                const YAML::Node ros2_node = n["ros2"];
                if (ros2_node && ros2_node["camera_topics"]) {
                    for (auto it = ros2_node["camera_topics"].begin(); it != ros2_node["camera_topics"].end(); ++it)
                        ros2_camera_topics[it->first.as<std::string>()] = it->second.as<std::string>();
                }
                if (ros2_node) {
                    if (ros2_node["use_ros2_bag"]) use_ros2_bag = ros2_node["use_ros2_bag"].as<bool>();
                    if (ros2_node["ros2_bag_file"]) ros2_bag_file = ros2_node["ros2_bag_file"].as<std::string>();
                    if (ros2_node["camera_topic"] && camera_tasks.empty()) {
                        const auto& ct = ros2_node["camera_topic"];
                        if (ct.IsSequence() && ct.size() > 0)
                            camera_topic = ct[0].as<std::string>();
                        else
                            camera_topic = ct.as<std::string>();
                    }
                    if (ros2_node["use_ros2_topics"]) use_ros2_realtime = ros2_node["use_ros2_topics"].as<bool>();
                    if (ros2_node["realtime_timeout"]) realtime_timeout_sec = ros2_node["realtime_timeout"].as<double>(60.0);
                    else if (ros2_node["max_wait_time"]) realtime_timeout_sec = ros2_node["max_wait_time"].as<double>(60.0);
                }

                for (const auto& s : n["sensors"]) {
                    if (s["type"] && s["type"].as<std::string>() == "camera") {
                        CameraTask task;
                        task.sensor_id = s["id"].as<std::string>();
                        task.camera_topic = s["topic"] ? s["topic"].as<std::string>() : "";
                        if (!ros2_camera_topics.empty()) {
                            auto it = ros2_camera_topics.find(task.sensor_id);
                            if (it != ros2_camera_topics.end())
                                task.camera_topic = it->second;
                        }
                        if (task.camera_topic.empty() && !camera_topic.empty())
                            task.camera_topic = camera_topic;
                        if (task.camera_topic.empty())
                            task.camera_topic = "/left/image_raw";
                        if (n["data"]["camera"] && n["data"]["camera"][task.sensor_id]) {
                            auto cam = n["data"]["camera"][task.sensor_id];
                            if (cam["images_dir"]) task.images_dir = cam["images_dir"].as<std::string>();
                            if (cam["reference_intrinsic_yaml"]) task.reference_intrinsic_yaml = cam["reference_intrinsic_yaml"].as<std::string>();
                        }
                        camera_tasks.push_back(task);
                    }
                }

                if (n["output_dir"]) output_dir = n["output_dir"].as<std::string>();
                if (n["results"] && n["results"]["camera_intrinsic"])
                    results_camera_intrinsic = n["results"]["camera_intrinsic"].as<std::string>();
                if (n["camera_intrinsic"]) {
                    const auto& ci = n["camera_intrinsic"];
                    if (ci["model"])        model_str = ci["model"].as<std::string>();
                    if (ci["min_images"])   cfg.min_images = ci["min_images"].as<int>();
                    if (ci["max_images"])   cfg.max_images = ci["max_images"].as<int>();
                    if (ci["max_rms_px"])   cfg.max_rms_px = ci["max_rms_px"].as<double>();
                    if (ci["refine_corners"]) cfg.refine_corners = ci["refine_corners"].as<bool>();
                    if (ci["subpix_max_iter"]) cfg.subpix_max_iter = ci["subpix_max_iter"].as<int>();
                    if (ci["subpix_epsilon"]) cfg.subpix_epsilon = ci["subpix_epsilon"].as<double>();
                    if (ci["outlier_rejection_sigma"]) cfg.outlier_rejection_sigma = ci["outlier_rejection_sigma"].as<double>();
                    if (ci["calibrate_max_iter"]) cfg.calibrate_max_iter = ci["calibrate_max_iter"].as<int>();
                    if (ci["calibrate_epsilon"]) cfg.calibrate_epsilon = ci["calibrate_epsilon"].as<double>();
                    if (ci["rational_model"]) cfg.rational_model = ci["rational_model"].as<bool>();
                    if (ci["prefer_no_checkerboard"]) prefer_no_checkerboard = ci["prefer_no_checkerboard"].as<bool>();
                    if (ci["target"]) {
                        auto t = ci["target"];
                        if (t["cols"])        cfg.target.cols  = t["cols"].as<int>();
                        if (t["rows"])        cfg.target.rows  = t["rows"].as<int>();
                        if (t["square_size"]) cfg.target.square_size_m = t["square_size"].as<double>();
                        if (t["type"]) {
                            std::string ts = t["type"].as<std::string>();
                            if (ts == "circles")         cfg.target.type = TargetConfig::Type::CIRCLES_GRID;
                            else if (ts == "asym_circles") cfg.target.type = TargetConfig::Type::ASYMMETRIC_CIRCLES;
                        }
                    }
                }
                if (n["third_party"]) {
                    const auto& tp = n["third_party"];
                    if (tp["dm_calib"]) dm_calib_repo = tp["dm_calib"].as<std::string>();
                    if (tp["dm_calib_model"]) dm_calib_model = tp["dm_calib_model"].as<std::string>();
                    if (tp["dm_calib_use_cpu"]) dm_calib_use_cpu = tp["dm_calib_use_cpu"].as<bool>();
                    if (tp["dm_calib_timeout_sec"]) dm_calib_timeout_sec = tp["dm_calib_timeout_sec"].as<int>();
                    if (tp["dm_calib_denoise_steps"]) dm_calib_denoise_steps = tp["dm_calib_denoise_steps"].as<int>();
                    if (tp["dm_calib_ensemble_size"]) dm_calib_ensemble_size = tp["dm_calib_ensemble_size"].as<int>();
                    if (tp["dm_calib_processing_res"]) dm_calib_processing_res = tp["dm_calib_processing_res"].as<int>();
                    if (tp["dm_calib_max_images"]) dm_calib_max_images = tp["dm_calib_max_images"].as<int>();
                }
                if (n["ai_root"]) config_ai_root = n["ai_root"].as<std::string>();

                // 兼容：若未从 sensors 解析到任何相机，则按“单相机”构造一项（用旧逻辑的 sensor_id/images_dir/camera_topic）
                if (camera_tasks.empty()) {
                    CameraTask single;
                    single.sensor_id = sensor_id;
                    single.camera_topic = camera_topic.empty() ? "/left/image_raw" : camera_topic;
                    single.images_dir = images_dir;
                    if (n["data"]["camera"] && n["data"]["camera"][sensor_id] && n["data"]["camera"][sensor_id]["reference_intrinsic_yaml"])
                        single.reference_intrinsic_yaml = n["data"]["camera"][sensor_id]["reference_intrinsic_yaml"].as<std::string>();
                    camera_tasks.push_back(single);
                }
            } else {
                if (n["images_dir"])   images_dir  = n["images_dir"].as<std::string>();
                if (n["model"])        model_str   = n["model"].as<std::string>();
                if (n["output_dir"])  output_dir  = n["output_dir"].as<std::string>();
                if (n["sensor_id"])   sensor_id   = n["sensor_id"].as<std::string>();
                if (n["target"]) {
                    auto t = n["target"];
                    if (t["cols"])        cfg.target.cols  = t["cols"].as<int>();
                    if (t["rows"])        cfg.target.rows  = t["rows"].as<int>();
                    if (t["square_size"]) cfg.target.square_size_m = t["square_size"].as<double>();
                    if (t["type"]) {
                        std::string ts = t["type"].as<std::string>();
                        if (ts == "circles")   cfg.target.type = TargetConfig::Type::CIRCLES_GRID;
                        else if (ts == "asym_circles") cfg.target.type = TargetConfig::Type::ASYMMETRIC_CIRCLES;
                    }
                }
                if (n["min_images"]) cfg.min_images = n["min_images"].as<int>();
                if (n["max_images"]) cfg.max_images = n["max_images"].as<int>();
                if (n["max_rms_px"]) cfg.max_rms_px = n["max_rms_px"].as<double>();
                if (n["refine_corners"]) cfg.refine_corners = n["refine_corners"].as<bool>();
                if (n["subpix_max_iter"]) cfg.subpix_max_iter = n["subpix_max_iter"].as<int>();
                if (n["subpix_epsilon"]) cfg.subpix_epsilon = n["subpix_epsilon"].as<double>();
                if (n["outlier_rejection_sigma"]) cfg.outlier_rejection_sigma = n["outlier_rejection_sigma"].as<double>();
                if (n["calibrate_max_iter"]) cfg.calibrate_max_iter = n["calibrate_max_iter"].as<int>();
                if (n["calibrate_epsilon"]) cfg.calibrate_epsilon = n["calibrate_epsilon"].as<double>();
                if (n["rational_model"]) cfg.rational_model = n["rational_model"].as<bool>();
                if (n["prefer_no_checkerboard"]) prefer_no_checkerboard = n["prefer_no_checkerboard"].as<bool>();
                if (n["third_party"]) {
                    const auto& tp = n["third_party"];
                    if (tp["dm_calib"]) dm_calib_repo = tp["dm_calib"].as<std::string>();
                    if (tp["dm_calib_model"]) dm_calib_model = tp["dm_calib_model"].as<std::string>();
                    if (tp["dm_calib_use_cpu"]) dm_calib_use_cpu = tp["dm_calib_use_cpu"].as<bool>();
                    if (tp["dm_calib_timeout_sec"]) dm_calib_timeout_sec = tp["dm_calib_timeout_sec"].as<int>();
                    if (tp["dm_calib_denoise_steps"]) dm_calib_denoise_steps = tp["dm_calib_denoise_steps"].as<int>();
                    if (tp["dm_calib_ensemble_size"]) dm_calib_ensemble_size = tp["dm_calib_ensemble_size"].as<int>();
                    if (tp["dm_calib_processing_res"]) dm_calib_processing_res = tp["dm_calib_processing_res"].as<int>();
                }
                if (n["ai_root"]) config_ai_root = n["ai_root"].as<std::string>();
                // 简配无 sensors：构造单相机任务
                if (camera_tasks.empty()) {
                    CameraTask single;
                    single.sensor_id = sensor_id;
                    single.images_dir = images_dir;
                    single.camera_topic = camera_topic.empty() ? "/left/image_raw" : camera_topic;
                    camera_tasks.push_back(single);
                }
            }
        } else if (arg == "--images_dir" && i + 1 < argc) {
            images_dir = argv[++i];
        } else if ((arg == "--data_dir" || arg == "--data-dir") && i + 1 < argc) {
            data_dir = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            model_str = argv[++i];
        } else if ((arg == "--output_dir" || arg == "--output-dir") && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--sensor_id" && i + 1 < argc) {
            sensor_id = argv[++i];
        } else if (arg == "--cols" && i + 1 < argc) {
            cfg.target.cols = std::stoi(argv[++i]);
        } else if (arg == "--rows" && i + 1 < argc) {
            cfg.target.rows = std::stoi(argv[++i]);
        } else if (arg == "--sq_size" && i + 1 < argc) {
            cfg.target.square_size_m = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: unicalib_camera_intrinsic [options]\n"
                      << "  --config <yaml>        配置文件 (支持 ros2.use_ros2_bag / use_ros2_topics)\n"
                      << "  --images_dir <dir>      棋盘格图像目录 (相对路径相对 --data-dir)\n"
                      << "  --data-dir <dir>        数据根目录 (解析相对路径与 bag 路径)\n"
                      << "  --output-dir <dir>      输出目录\n"
                      << "  --model pinhole|fisheye --sensor_id <id>\n"
                      << "  --cols/--rows/--sq_size 标定板参数\n"
                      << "数据源: 图像目录 | ROS2 bag | 实时话题 (与 IMU 内参一致)\n";
            return 0;
        }
    }

    // 未从 config 解析到任何相机时，按单相机构造一项（兼容仅 --images_dir / --sensor_id 等）
    if (camera_tasks.empty()) {
        CameraTask single;
        single.sensor_id = sensor_id;
        single.images_dir = images_dir;
        single.camera_topic = camera_topic.empty() ? "/left/image_raw" : camera_topic;
        camera_tasks.push_back(single);
    }

    if (data_dir.empty()) {
        const char* env_data = std::getenv("CALIB_DATA_DIR");
        if (env_data && env_data[0]) data_dir = env_data;
    }
    if (dm_calib_repo.empty()) {
        const char* env_dm = std::getenv("UNICALIB_DM_CALIB");
        if (env_dm && env_dm[0]) dm_calib_repo = env_dm;
    }
    if (dm_calib_model.empty()) {
        const char* env_model = std::getenv("UNICALIB_DM_CALIB_MODEL");
        if (env_model && env_model[0]) dm_calib_model = env_model;
    }
    // 容器内 calib_unified_run.sh 会设置 AI_ROOT；或配置中 ai_root；无棋盘格标定用其下的 DM-Calib
    if (dm_calib_repo.empty()) {
        const char* ai_root = std::getenv("AI_ROOT");
        if (ai_root && ai_root[0]) {
            dm_calib_repo = std::string(ai_root) + "/DM-Calib";
            if (dm_calib_model.empty()) dm_calib_model = dm_calib_repo + "/model";
        } else if (!config_ai_root.empty()) {
            dm_calib_repo = config_ai_root + "/DM-Calib";
            if (dm_calib_model.empty()) dm_calib_model = dm_calib_repo + "/model";
        }
    }

    // 非 ROS2 模式：解析每个任务的 images_dir / reference_intrinsic_yaml（相对 data_dir）
    for (auto& t : camera_tasks) {
        if (!t.images_dir.empty() && !data_dir.empty()) {
            fs::path p(t.images_dir);
            if (!p.is_absolute())
                t.images_dir = resolve_data_path(data_dir, t.images_dir);
        }
        if (!t.reference_intrinsic_yaml.empty() && !data_dir.empty()) {
            fs::path p(t.reference_intrinsic_yaml);
            if (!p.is_absolute())
                t.reference_intrinsic_yaml = resolve_data_path(data_dir, t.reference_intrinsic_yaml);
        }
    }

    bool need_images_dir = !use_ros2_bag && !use_ros2_realtime;
    if (need_images_dir) {
        bool any_has_dir = false;
        for (const auto& t : camera_tasks) {
            if (!t.images_dir.empty()) any_has_dir = true;
        }
        if (!any_has_dir) {
            ns_unicalib::Logger::init("UniCalib-Camera-Intrinsic");
            UNICALIB_ERROR("No images source: set ros2.use_ros2_bag + ros2.ros2_bag_file (and --data-dir), or data.camera.<id>.images_dir + --data-dir");
            return 1;
        }
        for (const auto& t : camera_tasks) {
            if (t.images_dir.empty()) continue;
            if (!fs::exists(t.images_dir) || !fs::is_directory(t.images_dir)) {
                ns_unicalib::Logger::init("UniCalib-Camera-Intrinsic");
                UNICALIB_ERROR("Images directory does not exist for {}: {} (若为相对路径请指定 --data-dir)", t.sensor_id, t.images_dir);
                return 1;
            }
        }
    }

    fs::create_directories(output_dir);
    std::string logs_dir = ns_unicalib::resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/camera_intrinsic_" + ns_unicalib::log_timestamp_filename() + ".log";
    ns_unicalib::Logger::init("UniCalib-Camera-Intrinsic", log_file, spdlog::level::info);
    UNICALIB_INFO("日志文件: {}", log_file);
    UNICALIB_INFO("待标定相机数: {} (配置中 sensors 内 type:camera 数量)", camera_tasks.size());

    if (model_str == "fisheye") {
        cfg.model = CameraIntrinsics::Model::FISHEYE;
    } else {
        cfg.model = CameraIntrinsics::Model::PINHOLE;
    }

    GlobalCamIntrinConfig g;
    g.output_dir = output_dir;
    g.data_dir = data_dir;
    g.results_camera_intrinsic = results_camera_intrinsic;
    g.model_str = model_str;
    g.prefer_no_checkerboard = prefer_no_checkerboard;
    g.cfg = cfg;
    g.dm_calib_repo = dm_calib_repo;
    g.dm_calib_model = dm_calib_model;
    g.dm_calib_use_cpu = dm_calib_use_cpu;
    g.dm_calib_timeout_sec = dm_calib_timeout_sec;
    g.dm_calib_denoise_steps = dm_calib_denoise_steps;
    g.dm_calib_ensemble_size = dm_calib_ensemble_size;
    g.dm_calib_processing_res = dm_calib_processing_res;
    g.dm_calib_max_images = dm_calib_max_images;

    UNICALIB_INFO("相机内参标定策略: {} (避免无棋盘格时标定失败)",
                  prefer_no_checkerboard ? "优先无棋盘格 (DM-Calib/先验)，棋盘格可选精化" : "仅棋盘格标定");

    int num_ok = 0;
    if (use_ros2_bag || use_ros2_realtime) {
#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
        std::string bag_path = ros2_bag_file;
        if (use_ros2_bag) {
            if (!bag_path.empty() && !data_dir.empty()) {
                fs::path p(bag_path);
                if (!p.is_absolute()) bag_path = resolve_data_path(data_dir, ros2_bag_file);
            }
            if (bag_path.empty()) {
                UNICALIB_ERROR("ROS2 bag 模式需配置 ros2.ros2_bag_file 并传入 --data-dir 或 CALIB_DATA_DIR");
                return 1;
            }
            if (!fs::exists(bag_path)) {
                UNICALIB_ERROR("ROS2 bag 路径不存在: {}", bag_path);
                return 1;
            }
        }
        ns_unicalib::RosDataSourceConfig ros_cfg;
        ros_cfg.realtime_mode = use_ros2_realtime;
        ros_cfg.bag_file = use_ros2_bag ? bag_path : "";
        for (const auto& t : camera_tasks) {
            ros_cfg.camera_topics[t.sensor_id] = t.camera_topic;
        }
        ros_cfg.camera_ros2_topic = camera_tasks.empty() ? "/left/image_raw" : camera_tasks.front().camera_topic;
        ros_cfg.realtime_timeout = realtime_timeout_sec;
        if (cfg.max_images > 0) ros_cfg.max_frames = static_cast<size_t>(cfg.max_images);

        auto ros_source = ns_unicalib::create_ros_data_source(ros_cfg);
        if (!ros_source) {
            UNICALIB_ERROR("创建 ROS2 数据源失败");
            return 1;
        }
        if (!ros_source->load()) {
            UNICALIB_ERROR("加载 ROS2 数据失败: {}", ros_source->get_status_message());
            return 1;
        }

        if (use_ros2_realtime) {
            auto* realtime_src = dynamic_cast<ns_unicalib::Ros2RealtimeDataSource*>(ros_source.get());
            if (realtime_src && !realtime_src->wait_for_data(std::min(10.0, realtime_timeout_sec * 0.2))) {
                UNICALIB_WARN("实时数据等待较短超时，继续采集...");
            }
            UNICALIB_INFO("实时采集图像中，超时 {:.0f} 秒...", realtime_timeout_sec);
            auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(realtime_timeout_sec);
            while (std::chrono::steady_clock::now() < deadline) {
                bool enough = true;
                for (const auto& t : camera_tasks) {
                    auto frames = ros_source->get_camera_frames(t.sensor_id);
                    if (ros_cfg.max_frames > 0 && frames.size() < ros_cfg.max_frames) enough = false;
                }
                if (enough) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        for (const auto& task : camera_tasks) {
            auto frames = ros_source->get_camera_frames(task.sensor_id);
            std::vector<cv::Mat> image_mats;
            int image_width = 0, image_height = 0;
            for (const auto& fr : frames) {
                if (!fr.image.empty()) {
                    image_mats.push_back(fr.image.clone());
                    if (image_width == 0) { image_width = fr.image.cols; image_height = fr.image.rows; }
                }
            }
            UNICALIB_INFO("从 ROS2 {} 获取 {} 帧图像 ({} 话题: {})",
                         use_ros2_bag ? "bag" : "实时", image_mats.size(), task.sensor_id, task.camera_topic);
            if (image_mats.size() < static_cast<size_t>(cfg.min_images)) {
                UNICALIB_ERROR("{} 图像不足: {} 帧 (需要 ≥ {})", task.sensor_id, image_mats.size(), cfg.min_images);
                continue;
            }
            int ret = run_single_camera(task, g, {}, std::move(image_mats), image_width, image_height);
            if (ret == 0) ++num_ok;
        }
        if (use_ros2_realtime) {
            auto* realtime_src = dynamic_cast<ns_unicalib::Ros2RealtimeDataSource*>(ros_source.get());
            if (realtime_src) realtime_src->stop();
        }
#else
        UNICALIB_ERROR("ROS2 数据源需要编译时启用 UNICALIB_WITH_ROS2。请使用图像目录或带 ROS2 的构建。");
        return 1;
#endif
    } else {
        for (const auto& task : camera_tasks) {
            if (task.images_dir.empty()) {
                UNICALIB_WARN("跳过 {}：未配置 images_dir", task.sensor_id);
                continue;
            }
            std::vector<std::string> image_paths = collect_images(task.images_dir);
            UNICALIB_INFO("从目录读取 {} 张图像 ({}): {}", image_paths.size(), task.sensor_id, task.images_dir);
            if (image_paths.size() < static_cast<size_t>(cfg.min_images)) {
                UNICALIB_ERROR("{} 图像不足: {} 张 (需要 ≥ {})", task.sensor_id, image_paths.size(), cfg.min_images);
                continue;
            }
            int ret = run_single_camera(task, g, std::move(image_paths), {}, 0, 0);
            if (ret == 0) ++num_ok;
        }
    }

    UNICALIB_INFO("相机内参标定结束: 成功 {} / {}", num_ok, camera_tasks.size());
    std::cout << "\n共标定 " << num_ok << " 个相机，结果见 " << (results_camera_intrinsic.empty() ? output_dir : output_dir + "/" + results_camera_intrinsic) << "/camera_intrinsic_<id>.yaml\n\n";

    main_exit = (num_ok == static_cast<int>(camera_tasks.size()) ? 0 : 1);
    UNICALIB_MAIN_TRY_END(main_exit)
}
