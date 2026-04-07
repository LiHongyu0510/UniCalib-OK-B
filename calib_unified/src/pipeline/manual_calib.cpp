/**
 * UniCalib Unified — ManualCalib 实现
 *
 * 手动标定的核心思路:
 *   1. ManualExtrinsicAdjuster:
 *      - 维护一个 ExtrinsicSE3 的可撤销栈
 *      - 每次 apply() 操作记录到 undo_stack_
 *      - 提供增量旋转/平移调整 (右乘 delta SE3)
 *
 *   2. ManualClickRefiner:
 *      - 收集用户在图像对上的点击对应点
 *      - 调用 Ceres 非线性最小二乘优化
 *      - 生成残差可视化 (热力图叠加到图像)
 *
 *   3. ManualCalibSession:
 *      - 包装 1 + 2, 管理会话历史
 *      - 自动保存到 JSON 文件
 */

#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/common/calib_stage.h"
#include "unicalib/viz/cloud_viewer.h"
#if UNICALIB_WITH_PANGOLIN
#include "unicalib/viz/manual_lidar_cam_window.h"
#include "unicalib/viz/manual_lidar_lidar_window.h"
#endif
#include <cstdlib>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/sphere_manifold.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ─────────────────────────────────────────────────────────────────────────────
// 工具: 时间戳字符串
// ─────────────────────────────────────────────────────────────────────────────
static std::string ts_str() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// Ceres 代价函数: 3D→2D 重投影误差 (用于 LiDAR-Cam 点击精化)
// ─────────────────────────────────────────────────────────────────────────────
struct LidarCamReprojCost {
    Eigen::Vector3d pt3d;
    Eigen::Vector2d pt2d;
    double fx, fy, cx, cy;

    LidarCamReprojCost(const Eigen::Vector3d& p3,
                        const Eigen::Vector2d& p2,
                        double fx_, double fy_, double cx_, double cy_)
        : pt3d(p3), pt2d(p2), fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

    template <typename T>
    bool operator()(const T* const rotation_aa,  // angle-axis [3]
                    const T* const translation,   // [3]
                    T* residuals) const {
        T p[3] = {T(pt3d.x()), T(pt3d.y()), T(pt3d.z())};
        T p_cam[3];
        ceres::AngleAxisRotatePoint(rotation_aa, p, p_cam);
        p_cam[0] += translation[0];
        p_cam[1] += translation[1];
        p_cam[2] += translation[2];

        // 投影
        T u = T(fx) * p_cam[0] / p_cam[2] + T(cx);
        T v = T(fy) * p_cam[1] / p_cam[2] + T(cy);

        residuals[0] = u - T(pt2d.x());
        residuals[1] = v - T(pt2d.y());
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p3,
                                        const Eigen::Vector2d& p2,
                                        double fx, double fy,
                                        double cx, double cy) {
        return new ceres::AutoDiffCostFunction<LidarCamReprojCost, 2, 3, 3>(
            new LidarCamReprojCost(p3, p2, fx, fy, cx, cy));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Ceres 代价函数: Cam-Cam 对极约束误差 (用于手动点击精化)
// ─────────────────────────────────────────────────────────────────────────────
struct CamCamEpipolarCost {
    Eigen::Vector2d pt0_norm;  // 归一化坐标
    Eigen::Vector2d pt1_norm;

    CamCamEpipolarCost(const Eigen::Vector2d& p0,
                        const Eigen::Vector2d& p1)
        : pt0_norm(p0), pt1_norm(p1) {}

    template <typename T>
    bool operator()(const T* const rotation_aa,
                    const T* const translation,
                    T* residuals) const {
        // t × (R*p0) · p1 = 0  (基本矩阵约束)
        T p0[3] = {T(pt0_norm.x()), T(pt0_norm.y()), T(1.0)};
        T Rp0[3];
        ceres::AngleAxisRotatePoint(rotation_aa, p0, Rp0);

        // t × Rp0
        T tx = translation[0], ty = translation[1], tz = translation[2];
        T cross[3] = {
            ty * Rp0[2] - tz * Rp0[1],
            tz * Rp0[0] - tx * Rp0[2],
            tx * Rp0[1] - ty * Rp0[0]
        };

        // · p1
        T p1[3] = {T(pt1_norm.x()), T(pt1_norm.y()), T(1.0)};
        residuals[0] = cross[0]*p1[0] + cross[1]*p1[1] + cross[2]*p1[2];
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& p0,
                                        const Eigen::Vector2d& p1) {
        return new ceres::AutoDiffCostFunction<CamCamEpipolarCost, 1, 3, 3>(
            new CamCamEpipolarCost(p0, p1));
    }
};

// ===========================================================================
// 6-DOF 交互循环 (OpenCV 窗口 + 键盘)
// ===========================================================================
namespace {
static bool manual_camcam_debug_enabled() {
    static bool enabled = []() {
        const char* v = std::getenv("UNICALIB_MANUAL_CAMCAM_DEBUG");
        if (v == nullptr) return false;
        std::string s(v);
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return (s == "1" || s == "true" || s == "on" || s == "yes");
    }();
    return enabled;
}

#define MANUAL_CAMCAM_DBG(...) \
    do { \
        if (manual_camcam_debug_enabled()) { UNICALIB_INFO(__VA_ARGS__); } \
    } while (0)

enum class EpilineFreezeMode {
    None = 0,
    FreezeLeft = 1,
    FreezeRight = 2
};

static const char* epiline_freeze_mode_name(EpilineFreezeMode mode) {
    switch (mode) {
        case EpilineFreezeMode::FreezeLeft: return "FreezeLeft";
        case EpilineFreezeMode::FreezeRight: return "FreezeRight";
        default: return "None";
    }
}

struct CamCamClickState {
    int img0_w = 0;
    int full_w = 0;
    int full_h = 0;
    bool has_pending_left = false;
    Eigen::Vector2d pending_left = Eigen::Vector2d::Zero();
    std::vector<ClickCorrespondence> clicks;
};

static void on_cam_cam_mouse(int event, int x, int y, int /*flags*/, void* userdata) {
    if (userdata == nullptr) return;
    auto* st = static_cast<CamCamClickState*>(userdata);
    if (st->full_w <= 0 || st->full_h <= 0) return;
    if (event != cv::EVENT_LBUTTONDOWN) return;

    // OpenCV mouse callback 坐标即当前显示图像坐标，直接使用可避免额外缩放误差。
    const int ox = x;
    const int oy = y;
    if (ox < 0 || oy < 0 || ox >= st->full_w || oy >= st->full_h) return;

    if (ox < st->img0_w) {
        st->pending_left = Eigen::Vector2d(static_cast<double>(ox), static_cast<double>(oy));
        st->has_pending_left = true;
        UNICALIB_INFO("[ManualCalib][Cam-Cam] 选中左图点: ({:.2f}, {:.2f})",
                     st->pending_left.x(), st->pending_left.y());
        return;
    }

    if (!st->has_pending_left) return;
    ClickCorrespondence cc;
    cc.pt_cam0 = st->pending_left;
    cc.pt_cam1 = Eigen::Vector2d(static_cast<double>(ox - st->img0_w), static_cast<double>(oy));
    cc.confidence = 1.0;
    st->clicks.push_back(cc);
    st->has_pending_left = false;
    UNICALIB_INFO("[ManualCalib][Cam-Cam] 新增点对 #{}: left=({:.2f},{:.2f}) right=({:.2f},{:.2f})",
                  st->clicks.size(),
                  cc.pt_cam0.x(), cc.pt_cam0.y(),
                  cc.pt_cam1.x(), cc.pt_cam1.y());
}

std::optional<ExtrinsicSE3> run_6dof_interactive_loop(
    ManualExtrinsicAdjuster& adjuster,
    const std::string& window_title) {

    const int win_w = 640;
    const int win_h = 380;
    cv::namedWindow(window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_title, win_w, win_h);

    bool accepted = false;
    while (true) {
        const ExtrinsicSE3& cur = adjuster.current();
        Eigen::Vector3d t = cur.POS_TargetInRef;
        Eigen::Vector3d rpy_rad = cur.SO3_TargetInRef.matrix().eulerAngles(0, 1, 2);
        double roll_deg = rpy_rad.x() * 180.0 / M_PI;
        double pitch_deg = rpy_rad.y() * 180.0 / M_PI;
        double yaw_deg = rpy_rad.z() * 180.0 / M_PI;

        cv::Mat canvas(win_h, win_w, CV_8UC3, cv::Scalar(40, 40, 40));
        int y = 28;
        auto put = [&](const std::string& s, int line = 0) {
            y = 28 + line * 22;
            cv::putText(canvas, s, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1);
        };
        put("T (m): " + std::to_string(t.x()).substr(0,8) + ", " + std::to_string(t.y()).substr(0,8) + ", " + std::to_string(t.z()).substr(0,8), 0);
        put("RPY (deg): " + std::to_string(roll_deg).substr(0,7) + ", " + std::to_string(pitch_deg).substr(0,7) + ", " + std::to_string(yaw_deg).substr(0,7), 1);
        put("Q/A Roll  W/S Pitch  E/D Yaw  |  R/F Tx  T/G Ty  Y/H Tz  |  U Undo", 3);
        put("Shift+key = 10x step   Enter = Accept   Esc = Cancel", 4);
        cv::imshow(window_title, canvas);

        int key = cv::waitKey(80);
        if (key < 0) continue;

        bool fast = (key & 0xFF) != key;  // 简化：不检测 Shift，可用配置步长
        char k = static_cast<char>(key & 0xFF);
        if (k == 'q') adjuster.apply(AdjustCmd::ROLL_PLUS,  false);
        if (k == 'a') adjuster.apply(AdjustCmd::ROLL_MINUS, false);
        if (k == 'w') adjuster.apply(AdjustCmd::PITCH_PLUS, false);
        if (k == 's') adjuster.apply(AdjustCmd::PITCH_MINUS, false);
        if (k == 'e') adjuster.apply(AdjustCmd::YAW_PLUS,   false);
        if (k == 'd') adjuster.apply(AdjustCmd::YAW_MINUS,  false);
        if (k == 'r') adjuster.apply(AdjustCmd::TX_PLUS,    false);
        if (k == 'f') adjuster.apply(AdjustCmd::TX_MINUS,    false);
        if (k == 't') adjuster.apply(AdjustCmd::TY_PLUS,     false);
        if (k == 'g') adjuster.apply(AdjustCmd::TY_MINUS,    false);
        if (k == 'y') adjuster.apply(AdjustCmd::TZ_PLUS,     false);
        if (k == 'h') adjuster.apply(AdjustCmd::TZ_MINUS,    false);
        if (k == 'u') adjuster.undo();
        if (key == 13)  { accepted = true; break; }  // Enter
        if (key == 27)  { break; }  // Esc
    }
    cv::destroyWindow(window_title);
    if (accepted) return adjuster.current();
    return std::nullopt;
}

// LiDAR-LiDAR 专用：显示两个三维点云并进行 6-DOF 微调
std::optional<ExtrinsicSE3> run_6dof_interactive_loop_lidar_lidar(
    ManualExtrinsicAdjuster& adjuster,
    const std::string& window_title,
    const LiDARScan& ref_scan,
    const LiDARScan& target_scan) {

    if (!ref_scan.cloud || ref_scan.cloud->empty() || !target_scan.cloud || target_scan.cloud->empty()) {
        UNICALIB_WARN("[ManualCalib] Ref 或 Target 点云为空，回退到仅 6-DOF 文字窗口");
        return run_6dof_interactive_loop(adjuster, window_title);
    }

    // 创建 3D 可视化窗口 (PCL/VTK)
    auto viewer_3d = CloudViewer::Create("LiDAR-LiDAR 3D Manual Calib", true);
    viewer_3d->add_cloud(ref_scan.cloud, "ref_cloud", ViewColor::Cyan(), 1.0f);
    
    // 设置初始视角
    viewer_3d->set_camera_position({0, 0, 30}, {0, 0, 1});

    const int win_w = 640;
    const int win_h = 400;
    cv::namedWindow(window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_title, win_w, win_h);

    UNICALIB_INFO("[ManualCalib] LiDAR-LiDAR 3D 交互窗口已启动。请在 3D 窗口观察对齐效果，在 2D 窗口看按键说明。");

    bool accepted = false;
    while (true) {
        const ExtrinsicSE3& cur = adjuster.current();
        
        // 更新 3D 窗口中 target 点云的位置
        // p_ref = T_TargetInRef * p_target
        viewer_3d->remove("target_cloud");
        viewer_3d->add_cloud_with_pose(target_scan.cloud, cur.SE3_TargetInRef(), "target_cloud", ViewColor::Red());
        viewer_3d->remove("target_frame");
        viewer_3d->add_coordinate_frame(cur.SE3_TargetInRef(), "target_frame", 0.5f);

        // 2D 文字控制面板
        cv::Mat canvas(win_h, win_w, CV_8UC3, cv::Scalar(40, 40, 40));
        
        Eigen::Vector3d t = cur.POS_TargetInRef;
        Eigen::Vector3d rpy_deg = cur.euler_deg();

        int y = 28;
        auto put = [&](const std::string& s, int line = 0, cv::Scalar col = cv::Scalar(220, 220, 220)) {
            y = 28 + line * 22;
            cv::putText(canvas, s, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, col, 1);
        };
        
        put("LiDAR-LiDAR Manual Adjust (6-DOF)", 0, cv::Scalar(0, 255, 255));
        put("Ref: " + cur.ref_sensor_id + " (Cyan)", 1, cv::Scalar(255, 255, 0));
        put("Target: " + cur.target_sensor_id + " (Red)", 2, cv::Scalar(0, 0, 255));
        
        put("T (m): " + std::to_string(t.x()).substr(0,8) + ", " + std::to_string(t.y()).substr(0,8) + ", " + std::to_string(t.z()).substr(0,8), 4);
        put("RPY (deg): " + std::to_string(rpy_deg.x()).substr(0,7) + ", " + std::to_string(rpy_deg.y()).substr(0,7) + ", " + std::to_string(rpy_deg.z()).substr(0,7), 5);
        
        put("Q/A Roll  W/S Pitch  E/D Yaw", 7);
        put("R/F Tx    T/G Ty     Y/H Tz", 8);
        put("U Undo    Enter Accept  Esc Cancel", 9);
        put("Shift+key = 10x step", 10);

        cv::imshow(window_title, canvas);

        int key = cv::waitKey(80);
        if (key < 0) {
            if (viewer_3d->is_stopped()) break;
            continue;
        }

        bool fast = (key & 0xFF) != key;
        char k = static_cast<char>(key & 0xFF);
        if (k == 'q') adjuster.apply(AdjustCmd::ROLL_PLUS,   fast);
        if (k == 'a') adjuster.apply(AdjustCmd::ROLL_MINUS,  fast);
        if (k == 'w') adjuster.apply(AdjustCmd::PITCH_PLUS,  fast);
        if (k == 's') adjuster.apply(AdjustCmd::PITCH_MINUS, fast);
        if (k == 'e') adjuster.apply(AdjustCmd::YAW_PLUS,    fast);
        if (k == 'd') adjuster.apply(AdjustCmd::YAW_MINUS,   fast);
        if (k == 'r') adjuster.apply(AdjustCmd::TX_PLUS,     fast);
        if (k == 'f') adjuster.apply(AdjustCmd::TX_MINUS,    fast);
        if (k == 't') adjuster.apply(AdjustCmd::TY_PLUS,     fast);
        if (k == 'g') adjuster.apply(AdjustCmd::TY_MINUS,    fast);
        if (k == 'y') adjuster.apply(AdjustCmd::TZ_PLUS,     fast);
        if (k == 'h') adjuster.apply(AdjustCmd::TZ_MINUS,    fast);
        if (k == 'u') adjuster.undo();
        if (key == 13) { accepted = true; break; }
        if (key == 27) break;
        
        if (viewer_3d->is_stopped()) break;
    }

    cv::destroyWindow(window_title);
    viewer_3d.reset();
    
    if (accepted) return adjuster.current();
    return std::nullopt;
}

// LiDAR-Camera 专用：点云叠加到图像上显示，实时观察对齐效果；同时打开 3D 点云窗口便于查看环境
std::optional<ExtrinsicSE3> run_6dof_interactive_loop_lidar_cam(
    ManualExtrinsicAdjuster& adjuster,
    const std::string& window_title,
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin) {

    if (!scan.cloud || scan.cloud->empty() || image.empty()) {
        UNICALIB_WARN("[ManualCalib] 点云或图像为空，回退到仅 6-DOF 文字窗口");
        return run_6dof_interactive_loop(adjuster, window_title);
    }

    const int img_w = image.cols;
    const int img_h = image.rows;
    const int W = cam_intrin.width > 0 ? cam_intrin.width : img_w;
    const int H = cam_intrin.height > 0 ? cam_intrin.height : img_h;
    const double fx = cam_intrin.fx;
    const double fy = cam_intrin.fy;
    const double cx = cam_intrin.cx;
    const double cy = cam_intrin.cy;

    // 在启动可视化窗口前，先保存初始状态的投影图和点云（应对可能的环境崩溃）
    {
        auto now = std::chrono::system_clock::now();
        auto now_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
        localtime_r(&now_t, &tm_buf);
        std::ostringstream oss;
        oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        std::string time_tag = oss.str() + "_initial";

        std::string save_dir = "./results/lidar_camera_extrinsic";
        fs::create_directories(save_dir);

        std::string img_path = save_dir + "/manual_projection_" + time_tag + ".png";
        std::string pcd_path = save_dir + "/manual_cloud_" + time_tag + ".pcd";
        std::string yaml_path = save_dir + "/manual_extrinsic_" + time_tag + ".yaml";

        Sophus::SE3d T_view_init = adjuster.current().SE3_TargetInRef().inverse();
        cv::Mat initial_overlay = LiDARProjectionViz::project_to_image(
            *scan.cloud, image, T_view_init,
            fx, fy, cx, cy, W, H,
            0.5, 50.0);

        cv::imwrite(img_path, initial_overlay);
        pcl::io::savePCDFileBinary(pcd_path, *scan.cloud);
        adjuster.save(yaml_path);

        UNICALIB_INFO("[ManualCalib] 交互窗口启动前已自动保存初始状态快照（备用）:");
        UNICALIB_INFO("  - 投影图: {}", img_path);
        UNICALIB_INFO("  - 点云:   {}", pcd_path);
        UNICALIB_INFO("  - 外参:   {}", yaml_path);
    }

    cv::namedWindow(window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_title, img_w, img_h);

    // 3D 点云窗口（PCL/VTK）：根据用户要求，由于某些环境下 VTK 容易闪退，彻底禁用 3D 窗口，仅保留 2D 投影微调
    const bool use_3d_viewer = false; 
    UNICALIB_INFO("[ManualCalib] 3D 点云窗口已禁用（避免 VTK 闪退），请在 2D 投影窗口进行微调。按下 'S' 键可手动保存当前微调结果图片和点云。");

    CloudViewer::Ptr viewer_3d;
    if (use_3d_viewer) {
        g_current_calib_stage = "manual_lidar_cam_3d_create";
        viewer_3d = CloudViewer::Create("LiDAR 3D Point Cloud (Manual Calib)", true);  // 重新启用 true: 已在 CloudViewer 内部修复线程模型
        viewer_3d->add_cloud(scan.cloud, "lidar_cloud", ViewColor::Cyan(), 1.2f);
    }

    g_current_calib_stage = "manual_lidar_cam_loop";
    UNICALIB_INFO("[ManualCalib] 初始位姿 T_lidar_to_cam:");
    {
        auto T = adjuster.current().SE3_TargetInRef();
        Eigen::Vector3d rpy = T.so3().matrix().eulerAngles(0, 1, 2) * 180.0 / M_PI;
        UNICALIB_INFO("  - Translation: {:.4f}, {:.4f}, {:.4f}", T.translation().x(), T.translation().y(), T.translation().z());
        UNICALIB_INFO("  - RPY (deg):   {:.4f}, {:.4f}, {:.4f}", rpy.x(), rpy.y(), rpy.z());
    }

    bool accepted = false;
    while (true) {
        const ExtrinsicSE3& cur = adjuster.current();
        // 恢复：经分析 project_to_image 需要的是相机在雷达系下的位姿 T_cam_in_lidar
        Sophus::SE3d T_view = cur.SE3_TargetInRef().inverse(); 

        if (viewer_3d) {
            viewer_3d->remove("lidar_cam_frame");
            viewer_3d->add_coordinate_frame(T_view, "lidar_cam_frame", 0.4f);
            // 3D 窗口由 CloudViewer 后台线程同步刷新，此处无需再调用 spin_once，减少主线程压力
        }

        cv::Mat overlay = LiDARProjectionViz::project_to_image(
            *scan.cloud, image, T_view,
            fx, fy, cx, cy, W, H,
            0.5, 50.0);

        Eigen::Vector3d t = cur.POS_TargetInRef;
        Eigen::Vector3d rpy_rad = cur.SO3_TargetInRef.matrix().eulerAngles(0, 1, 2);
        double roll_deg = rpy_rad.x() * 180.0 / M_PI;
        double pitch_deg = rpy_rad.y() * 180.0 / M_PI;
        double yaw_deg = rpy_rad.z() * 180.0 / M_PI;

        int y = 24;
        auto put = [&](const std::string& s, int line = 0) {
            y = 24 + line * 20;
            cv::putText(overlay, s, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        };
        put("T(m): " + std::to_string(t.x()).substr(0, 7) + " " + std::to_string(t.y()).substr(0, 7) + " " + std::to_string(t.z()).substr(0, 7), 0);
        put("RPY(deg): " + std::to_string(roll_deg).substr(0, 6) + " " + std::to_string(pitch_deg).substr(0, 6) + " " + std::to_string(yaw_deg).substr(0, 6), 1);
        put("Q/A Roll  W/S Pitch  E/D Yaw  R/F Tx  T/G Ty  Y/H Tz  U Undo", 3);
        put("Enter=Accept  Esc=Cancel  S=Save Current Snapshot", 4);

        cv::imshow(window_title, overlay);

        // 3D 窗口由 CloudViewer 后台线程刷新，主线程不再调用 spin_once，两窗可同时稳定显示

        int key = cv::waitKey(80);
        if (key < 0) continue;

        bool fast = (key & 0xFF) != key;
        char k = static_cast<char>(key & 0xFF);
        if (k == 'q') adjuster.apply(AdjustCmd::ROLL_PLUS,   false);
        if (k == 'a') adjuster.apply(AdjustCmd::ROLL_MINUS,  false);
        if (k == 'w') adjuster.apply(AdjustCmd::PITCH_PLUS,  false);
        if (k == 's') adjuster.apply(AdjustCmd::PITCH_MINUS, false);
        if (k == 'e') adjuster.apply(AdjustCmd::YAW_PLUS,    false);
        if (k == 'd') adjuster.apply(AdjustCmd::YAW_MINUS,  false);
        if (k == 'r') adjuster.apply(AdjustCmd::TX_PLUS,    false);
        if (k == 'f') adjuster.apply(AdjustCmd::TX_MINUS,   false);
        if (k == 't') adjuster.apply(AdjustCmd::TY_PLUS,    false);
        if (k == 'g') adjuster.apply(AdjustCmd::TY_MINUS,  false);
        if (k == 'y') adjuster.apply(AdjustCmd::TZ_PLUS,    false);
        if (k == 'h') adjuster.apply(AdjustCmd::TZ_MINUS,   false);
        if (k == 'u') adjuster.undo();
        if (k == 'S' || k == 's' || key == 'S') {
            // 保存当前微调结果
            auto now = std::chrono::system_clock::now();
            auto now_t = std::chrono::system_clock::to_time_t(now);
            std::tm tm_buf{};
            localtime_r(&now_t, &tm_buf);
            std::ostringstream oss;
            oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
            std::string time_tag = oss.str();

            std::string save_dir = "./results/lidar_camera_extrinsic";
            fs::create_directories(save_dir);

            std::string img_path = save_dir + "/manual_projection_" + time_tag + ".png";
            std::string pcd_path = save_dir + "/manual_cloud_" + time_tag + ".pcd";
            std::string yaml_path = save_dir + "/manual_extrinsic_" + time_tag + ".yaml";

            cv::imwrite(img_path, overlay);
            pcl::io::savePCDFileBinary(pcd_path, *scan.cloud);
            adjuster.save(yaml_path);

            UNICALIB_INFO("[ManualCalib] 手动微调快照已保存:");
            UNICALIB_INFO("  - 投影图: {}", img_path);
            UNICALIB_INFO("  - 点云:   {}", pcd_path);
            UNICALIB_INFO("  - 外参:   {}", yaml_path);
        }
        if (key == 13) { accepted = true; break; }
        if (key == 27) { break; }
    }
    cv::destroyWindow(window_title);
    if (viewer_3d) viewer_3d.reset();
    g_current_calib_stage = nullptr;
    if (accepted) {
        // 最终接受时也保存一份，确保用户微调的结果被记录
        auto now = std::chrono::system_clock::now();
        auto now_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
        localtime_r(&now_t, &tm_buf);
        std::ostringstream oss;
        oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        std::string time_tag = oss.str() + "_final";

        std::string save_dir = "./results/lidar_camera_extrinsic";
        fs::create_directories(save_dir);

        std::string img_path = save_dir + "/manual_projection_" + time_tag + ".png";
        std::string pcd_path = save_dir + "/manual_cloud_" + time_tag + ".pcd";
        std::string yaml_path = save_dir + "/manual_extrinsic_" + time_tag + ".yaml";

        // 恢复：获取当前相机在雷达系下的位姿并重新生成投影图
        Sophus::SE3d T_view_final = adjuster.current().SE3_TargetInRef().inverse();
        cv::Mat final_overlay = LiDARProjectionViz::project_to_image(
            *scan.cloud, image, T_view_final,
            fx, fy, cx, cy, W, H,
            0.5, 50.0);

        cv::imwrite(img_path, final_overlay);
        pcl::io::savePCDFileBinary(pcd_path, *scan.cloud);
        adjuster.save(yaml_path);

        UNICALIB_INFO("[ManualCalib] 最终微调结果已自动保存:");
        UNICALIB_INFO("  - 投影图: {}", img_path);
        UNICALIB_INFO("  - 点云:   {}", pcd_path);
        UNICALIB_INFO("  - 外参:   {}", yaml_path);

        return adjuster.current();
    }
    return std::nullopt;
}

// 水平扫描参考线：固定步长（与原始对极采样 80px 一致），每条线不同颜色便于对齐行号
static cv::Scalar horizontal_guide_color_bgr(size_t line_index, int hue_offset_deg) {
    int h = (hue_offset_deg + static_cast<int>((line_index * 47) % 180) + 180) % 180;
    cv::Mat hsv(1, 1, CV_8UC3);
    hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(static_cast<unsigned char>(h), 200, 255);
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b v = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(v[0], v[1], v[2]);
}

static void draw_horizontal_guides_rainbow(cv::Mat& vis, int period_px, int hue_offset_deg) {
    if (vis.empty() || period_px < 16) return;
    size_t idx = 0;
    for (int y = period_px; y < vis.rows - period_px; y += period_px, ++idx) {
        cv::line(vis, cv::Point(0, y), cv::Point(vis.cols - 1, y),
                 horizontal_guide_color_bgr(idx, hue_offset_deg), 1, cv::LINE_AA);
    }
}

// 极点（齐次）：F*e0=0（左图）、F^T*e1=0（右图）。以极点为圆心的同心圆与每条对极线相交于距极点定长两点，
// 形成极坐标式网格，便于沿极线定位对应点（参见多视图几何中极点–极线关系）。
static bool epipole_homogeneous_to_pixel(const Eigen::Vector3d& e, double& px, double& py) {
    if (std::abs(e[2]) < 1e-6) return false;
    px = e[0] / e[2];
    py = e[1] / e[2];
    return std::isfinite(px) && std::isfinite(py);
}

// 点 (px,py) 到图像矩形 [0,W]×[0,H]（与 OpenCV 画布一致）的最短距离；在矩形内为 0。
// 用于同心圆：从「第一圈与图像区域相交」的半径起画，避免圆心在图外时前几圈完全落在画布外。
static double distance_point_to_image_rect(double px, double py, int cols, int rows) {
    if (cols <= 0 || rows <= 0) return 0.0;
    const double cx = std::clamp(px, 0.0, static_cast<double>(cols));
    const double cy = std::clamp(py, 0.0, static_cast<double>(rows));
    return std::hypot(px - cx, py - cy);
}

// 圆心在极点：半径沿极线方向，圆在交点处的切线与极线垂直（与极坐标中 r=const 与 θ=const 正交一致）。
// 绘制垂直于对极线的参考线。
// 情况 A: 极点在有限距离内 -> 绘制以极点为圆心的圆（切线与极线垂直）。
// 情况 B: 极点在无穷远（近似平行双目） -> 绘制垂直于平行对极线束的平行线。
//
// 情况 B 几何：对极线 ax+by+c=0 的法向为 (a,b)、切向为 (-b,a)。与对极线垂直的直线方向应与切向垂直，
// 即与法向 (a,b) 平行，故该直线的法向须为 (-b,a)（与 (a,b) 垂直）。切勿用 (a,b) 画 x,y 系数，
// 否则得到的是与对极线平行的直线族（此前 bug：与彩虹横线同向、看似「未画」）。
static void draw_perpendicular_guides(
    cv::Mat& vis, const Eigen::Vector3d& epipole_h,
    const std::vector<cv::Vec3f>& epilines,
    int step_px, int hue_offset_deg, int cross_view_locked_count = -1) {

    if (vis.empty() || step_px < 8) {
        MANUAL_CAMCAM_DBG("[GuideLog] 图像为空或步长太小: step={}", step_px);
        return;
    }

    MANUAL_CAMCAM_DBG("[GuideLog] 开始绘制垂直参考线. vis={}x{} step_px={} Epilines数={} 极点齐次=[{:.4f},{:.4f},{:.4f}]",
                      vis.cols, vis.rows, step_px, epilines.size(), epipole_h[0], epipole_h[1], epipole_h[2]);

    double ex = 0, ey = 0;
    bool finite_epipole = false;
    // 仅用「|ex|,|ey| < 15×宽高」会把极点在图像左侧/右侧很远但仍为有限齐次坐标的情况判成
    // “有限极点”，随后画以该点为圆心、r=80..2400 的小圆——圆与图像根本不相交（日志中右图即此）。
    // 只有极点落在图像邻域内时才用同心圆；否则与 e[2]≈0 一样走平行线束（情况 B）。
    const double img_margin = 0.5 * static_cast<double>(std::max(vis.cols, vis.rows));
    if (epipole_homogeneous_to_pixel(epipole_h, ex, ey)) {
        if (ex >= -img_margin && ex <= static_cast<double>(vis.cols) + img_margin &&
            ey >= -img_margin && ey <= static_cast<double>(vis.rows) + img_margin) {
            finite_epipole = true;
        }
        MANUAL_CAMCAM_DBG(
            "[GuideLog] 极点像素=({:.1f},{:.1f}) 视域邻域判定: margin={:.1f} 范围x∈[{:.1f},{:.1f}] y∈[{:.1f},{:.1f}] -> {}",
            ex, ey, img_margin, -img_margin, static_cast<double>(vis.cols) + img_margin, -img_margin,
            static_cast<double>(vis.rows) + img_margin, finite_epipole);
        MANUAL_CAMCAM_DBG("[GuideLog] 极点像素坐标: ({:.1f}, {:.1f}), 是否判定为有限且在视域邻域内: {}", ex, ey,
                          finite_epipole);
    } else {
        MANUAL_CAMCAM_DBG("[GuideLog] 极点在无穷远 (e[2]={:.8f})", epipole_h[2]);
    }

    bool drew_circles = false;
    if (finite_epipole) {
        double max_r = std::hypot(ex - 0, ey - 0);
        max_r = std::max(max_r, std::hypot(ex - vis.cols, ey - 0));
        max_r = std::max(max_r, std::hypot(ex - 0, ey - vis.rows));
        max_r = std::max(max_r, std::hypot(ex - vis.cols, ey - vis.rows));
        const double r_touch = distance_point_to_image_rect(ex, ey, vis.cols, vis.rows);
        int r = step_px;
        if (r_touch > 1e-3) {
            r = static_cast<int>(std::ceil(r_touch / static_cast<double>(step_px))) * step_px;
        }
        if (r > max_r + step_px) {
            UNICALIB_WARN(
                "[GuideLog] 情况A 放弃: 首圈半径 {} 已超过覆盖角点所需 {} (极点相对图像位置异常)，改绘平行线",
                r, max_r);
            finite_epipole = false;
        } else {
            MANUAL_CAMCAM_DBG("[GuideLog] 情况A (圆弧): 与图像相交最小半径≈{:.1f}, 最大半径={:.1f}, 首圈r={}, 步长={}",
                              r_touch, max_r, r, step_px);
            size_t count = 0;
            if (cross_view_locked_count > 0) {
                const double r_start = static_cast<double>(r);
                const double r_end = max_r;
                for (int i = 0; i < cross_view_locked_count; ++i) {
                    const double alpha = static_cast<double>(i + 1) /
                                         static_cast<double>(cross_view_locked_count + 1);
                    const int rr = static_cast<int>(std::round(r_start + alpha * (r_end - r_start)));
                    cv::Scalar col = horizontal_guide_color_bgr(static_cast<size_t>(i), hue_offset_deg);
                    cv::circle(vis, cv::Point(static_cast<int>(ex + 0.5), static_cast<int>(ey + 0.5)),
                               rr, col, 2, cv::LINE_AA);
                    ++count;
                }
                MANUAL_CAMCAM_DBG("[GuideLog] 情况A 跨图锁定色标模式: 绘制 {} 圈同心圆", count);
            } else {
                const size_t kMaxCircles = 512;
                while (r <= max_r + step_px && count < kMaxCircles) {
                    cv::Scalar col = horizontal_guide_color_bgr(count, hue_offset_deg);
                    cv::circle(vis, cv::Point(static_cast<int>(ex + 0.5), static_cast<int>(ey + 0.5)),
                               r, col, 2, cv::LINE_AA);
                    r += step_px; ++count;
                }
            }
            MANUAL_CAMCAM_DBG("[GuideLog] 绘制了 {} 圈同心圆", count);
            drew_circles = true;
        }
    }
    if (!drew_circles && !epilines.empty()) {
        // 与对极线垂直的直线：法向 g = (-b, a)，其中 (a,b) 为对极线已单位化法向。
        double sum_gx = 0, sum_gy = 0;
        int valid_lines = 0;
        double sample_a = 0, sample_b = 0;
        for (const auto& l : epilines) {
            double len = std::hypot(l[0], l[1]);
            if (len < 1e-7) continue;
            double a = l[0] / len;
            double b = l[1] / len;
            if (valid_lines == 0) {
                sample_a = a;
                sample_b = b;
            }
            double gx = -b;
            double gy = a;
            if (gx < 0 || (std::abs(gx) < 1e-7 && gy < 0)) {
                gx = -gx;
                gy = -gy;
            }
            sum_gx += gx;
            sum_gy += gy;
            valid_lines++;
        }

        if (valid_lines == 0) {
            UNICALIB_WARN("[GuideLog] 无有效对极线法向量");
            return;
        }

        double avg_gx = sum_gx / valid_lines;
        double avg_gy = sum_gy / valid_lines;
        double avg_len = std::hypot(avg_gx, avg_gy);
        MANUAL_CAMCAM_DBG("[GuideLog] 情况B: 对极线平均法向(样本)=[{:.4f},{:.4f}] 垂直参考线法向(未归一)=[{:.4f},{:.4f}] 有效线数={}",
                          sample_a, sample_b, avg_gx, avg_gy, valid_lines);

        if (avg_len > 1e-4) {
            avg_gx /= avg_len;
            avg_gy /= avg_len;
            const double kRadToDeg = 180.0 / M_PI;
            // 对极线切向 T=(-b,a)；垂直参考线 g·x=d 中 g=(-b,a)，沿参考线方向 v=(-g_y,g_x)
            const double epiline_tangent_deg = std::atan2(sample_a, -sample_b) * kRadToDeg;
            const double perp_guide_along_deg = std::atan2(-avg_gy, avg_gx) * kRadToDeg;
            const double ang_vs_horizontal =
                std::abs(perp_guide_along_deg);
            const double ang_wrap = std::min(ang_vs_horizontal, 180.0 - ang_vs_horizontal);
            MANUAL_CAMCAM_DBG(
                "[GuideLog] 几何: 对极线切向≈{:.1f}° | 垂直参考线走向≈{:.1f}° | 与+水平轴夹角≈{:.1f}° (旧版误用对极线法向画线时≈切向≈{:.1f}°，易与彩虹横线重合)",
                epiline_tangent_deg, perp_guide_along_deg, ang_wrap, epiline_tangent_deg);
            MANUAL_CAMCAM_DBG(
                "[GuideLog] 情况B 诊断: 对极线切向≈{:.1f}° 垂直参考走向≈{:.1f}° 与水平夹角≈{:.1f}°",
                epiline_tangent_deg, perp_guide_along_deg, ang_wrap);

            double d_min = 1e10, d_max = -1e10;
            const cv::Point2f pts[4] = {{0, 0},
                                        {(float)vis.cols, 0},
                                        {0, (float)vis.rows},
                                        {(float)vis.cols, (float)vis.rows}};
            for (int i = 0; i < 4; ++i) {
                double d = pts[i].x * avg_gx + pts[i].y * avg_gy;
                d_min = std::min(d_min, d);
                d_max = std::max(d_max, d);
            }
            MANUAL_CAMCAM_DBG("[GuideLog] 垂直参考 投影 d=gx*x+gy*y 范围: [{:.1f}, {:.1f}]", d_min, d_max);

            // 与彩虹横线、绿/橙极线区分：色相整体偏移 + 每条线不同色（细线）
            constexpr int kPerpHueOffset = 150;
            const int perp_hue_base = hue_offset_deg + kPerpHueOffset;
            size_t draw_count = 0;
            auto draw_line_d = [&](double d) {
                cv::Point2f p1, p2;
                if (std::abs(avg_gy) > std::abs(avg_gx)) {
                    p1 = {0, (float)(d / avg_gy)};
                    p2 = {(float)vis.cols, (float)((d - avg_gx * vis.cols) / avg_gy)};
                } else {
                    p1 = {(float)(d / avg_gx), 0};
                    p2 = {(float)((d - avg_gy * vis.rows) / avg_gx), (float)vis.rows};
                }
                MANUAL_CAMCAM_DBG("[GuideLog] 垂直参考线段#{} d={:.1f} p1=({:.1f},{:.1f}) p2=({:.1f},{:.1f})", draw_count,
                                  d, p1.x, p1.y, p2.x, p2.y);
                cv::Scalar col = horizontal_guide_color_bgr(draw_count, perp_hue_base);
                cv::line(vis, p1, p2, col, 1, cv::LINE_AA);
                ++draw_count;
            };
            if (cross_view_locked_count > 0 && d_max > d_min + 1e-9) {
                for (int i = 0; i < cross_view_locked_count; ++i) {
                    const double alpha = static_cast<double>(i + 1) /
                                         static_cast<double>(cross_view_locked_count + 1);
                    const double d = d_min + alpha * (d_max - d_min);
                    draw_line_d(d);
                }
                MANUAL_CAMCAM_DBG("[GuideLog] 情况B 跨图锁定色标模式: 绘制 {} 条垂直参考线", draw_count);
            } else {
                for (double d = d_min + step_px; d < d_max; d += step_px) {
                    draw_line_d(d);
                }
                if (draw_count == 0 && d_max > d_min + 1e-9) {
                    draw_line_d(0.5 * (d_min + d_max));
                    MANUAL_CAMCAM_DBG("[GuideLog] 投影跨距 {} 小于步长 {}，已补画一条中线", d_max - d_min, step_px);
                }
            }
            MANUAL_CAMCAM_DBG("[GuideLog] 绘制了 {} 条垂直于对极线的参考线 (1px 彩色)", draw_count);
        } else {
            MANUAL_CAMCAM_DBG("[GuideLog] 垂直参考法向平均长度过小 ({:.8f}), 无法绘制", avg_len);
        }
    }
}

// 对极线可视化：由 R,t 与 K0,K1 构造 F，在左右图上绘制极线并水平拼接
static cv::Mat render_stereo_epipolar_impl(
    const cv::Mat& img0,
    const cv::Mat& img1,
    const std::vector<ClickCorrespondence>& clicks,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin,
    EpilineFreezeMode freeze_mode = EpilineFreezeMode::None,
    const ExtrinsicSE3* anchor_extrin = nullptr) {

    if (img0.empty() || img1.empty()) return cv::Mat();

    cv::Mat vis0 = img0.clone();
    cv::Mat vis1 = img1.clone();
    if (vis0.channels() == 1) cv::cvtColor(vis0, vis0, cv::COLOR_GRAY2BGR);
    if (vis1.channels() == 1) cv::cvtColor(vis1, vis1, cv::COLOR_GRAY2BGR);

    const int kGuideStep = 80;  // 与下方对极网格步长一致，避免过密
    // 彩虹横线为屏幕行辅助，一般不与极线垂直；勿与以极点为心的圆混淆。
    draw_horizontal_guides_rainbow(vis0, kGuideStep, 0);
    draw_horizontal_guides_rainbow(vis1, kGuideStep, 72);

    Eigen::Matrix3d K0 = intrin0.K();
    Eigen::Matrix3d K1 = intrin1.K();
    auto calc_F = [&](const ExtrinsicSE3& e) {
        Eigen::Matrix3d R = e.SO3_TargetInRef.matrix();
        Eigen::Vector3d t = e.POS_TargetInRef;
        Eigen::Matrix3d t_skew_local;
        t_skew_local << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
        Eigen::Matrix3d E = t_skew_local * R;
        return K1.transpose().inverse() * E * K0.inverse();
    };
    const Eigen::Matrix3d F_cur = calc_F(extrin);
    const Eigen::Matrix3d F_anchor = (anchor_extrin != nullptr) ? calc_F(*anchor_extrin) : F_cur;
    const Eigen::Matrix3d& F_for_left = (freeze_mode == EpilineFreezeMode::FreezeLeft) ? F_anchor : F_cur;
    const Eigen::Matrix3d& F_for_right = (freeze_mode == EpilineFreezeMode::FreezeRight) ? F_anchor : F_cur;
    cv::Mat F_cv_cur, F_cv_left, F_cv_right;
    cv::eigen2cv(F_cur, F_cv_cur);
    cv::eigen2cv(F_for_left, F_cv_left);
    cv::eigen2cv(F_for_right, F_cv_right);

    const int grid_step = kGuideStep;
    std::vector<cv::Point2f> pts0_grid;
    for (int y = grid_step; y < vis0.rows - grid_step; y += grid_step)
        for (int x = grid_step; x < vis0.cols - grid_step; x += grid_step)
            pts0_grid.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));

    std::vector<cv::Vec3f> lines1;
    if (!pts0_grid.empty()) {
        cv::Mat pts0_mat(pts0_grid);
        cv::computeCorrespondEpilines(pts0_mat, 1, F_cv_right, lines1);
        for (size_t i = 0; i < lines1.size(); ++i) {
            const cv::Vec3f& l = lines1[i];
            float a = l[0], b = l[1], c = l[2];
            cv::Point2f p1(0, -c / b), p2(static_cast<float>(vis1.cols), -(a * vis1.cols + c) / b);
            if (std::abs(b) < 1e-6f) {
                p1 = cv::Point2f(-c / a, 0);
                p2 = cv::Point2f(-c / a, static_cast<float>(vis1.rows));
            }
            cv::line(vis1, p1, p2, cv::Scalar(0, 255, 0), 1);
        }
    }

    std::vector<cv::Point2f> pts1_grid;
    for (int y = grid_step; y < vis1.rows - grid_step; y += grid_step)
        for (int x = grid_step; x < vis1.cols - grid_step; x += grid_step)
            pts1_grid.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));

    std::vector<cv::Vec3f> lines0;
    if (!pts1_grid.empty()) {
        cv::Mat pts1_mat(pts1_grid);
        cv::computeCorrespondEpilines(pts1_mat, 2, F_cv_left, lines0);
        for (size_t i = 0; i < lines0.size(); ++i) {
            const cv::Vec3f& l = lines0[i];
            float a = l[0], b = l[1], c = l[2];
            cv::Point2f p1(0, -c / b), p2(static_cast<float>(vis0.cols), -(a * vis0.cols + c) / b);
            if (std::abs(b) < 1e-6f) {
                p1 = cv::Point2f(-c / a, 0);
                p2 = cv::Point2f(-c / a, static_cast<float>(vis0.rows));
            }
            cv::line(vis0, p1, p2, cv::Scalar(255, 128, 0), 1);
        }
    }

    // 从 F 计算极点（齐次坐标）：F*e0=0 (右零空间), F^T*e1=0 (左零空间)
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_left(F_for_left, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d e0_h = svd_left.matrixV().col(2);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_right(F_for_right, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d e1_h = svd_right.matrixU().col(2);

    // 绘制垂直参考线（圆或垂直平行线）
    // 跨图锁定色标：左右使用同一色相基准与同一序号数量，同色即同序号参考层。
    const int kCrossViewLockedGuideCount = 12;
    const int kCrossViewGuideHueBase = 30;
    draw_perpendicular_guides(vis0, e0_h, lines0, kGuideStep, kCrossViewGuideHueBase, kCrossViewLockedGuideCount);
    draw_perpendicular_guides(vis1, e1_h, lines1, kGuideStep, kCrossViewGuideHueBase, kCrossViewLockedGuideCount);

    for (const auto& cc : clicks) {
        cv::circle(vis0, cv::Point2d(cc.pt_cam0.x(), cc.pt_cam0.y()), 4, cv::Scalar(0, 0, 255), 2);
        cv::circle(vis1, cv::Point2d(cc.pt_cam1.x(), cc.pt_cam1.y()), 4, cv::Scalar(0, 0, 255), 2);
        std::vector<cv::Point2f> p0 = {cv::Point2f(static_cast<float>(cc.pt_cam0.x()), static_cast<float>(cc.pt_cam0.y()))};
        std::vector<cv::Point2f> p1 = {cv::Point2f(static_cast<float>(cc.pt_cam1.x()), static_cast<float>(cc.pt_cam1.y()))};
        std::vector<cv::Vec3f> l1, l0;
        cv::computeCorrespondEpilines(cv::Mat(p0), 1, F_cv_cur, l1);
        cv::computeCorrespondEpilines(cv::Mat(p1), 2, F_cv_cur, l0);
        if (!l1.empty()) {
            float a = l1[0][0], b = l1[0][1], c = l1[0][2];
            cv::Point2f pa(0, -c / b), pb(static_cast<float>(vis1.cols), -(a * vis1.cols + c) / b);
            if (std::abs(b) < 1e-6f) { pa = cv::Point2f(-c / a, 0); pb = cv::Point2f(-c / a, static_cast<float>(vis1.rows)); }
            cv::line(vis1, pa, pb, cv::Scalar(0, 255, 255), 2);
        }
        if (!l0.empty()) {
            float a = l0[0][0], b = l0[0][1], c = l0[0][2];
            cv::Point2f pa(0, -c / b), pb(static_cast<float>(vis0.cols), -(a * vis0.cols + c) / b);
            if (std::abs(b) < 1e-6f) { pa = cv::Point2f(-c / a, 0); pb = cv::Point2f(-c / a, static_cast<float>(vis0.rows)); }
            cv::line(vis0, pa, pb, cv::Scalar(0, 255, 255), 2);
        }
    }

    int w = vis0.cols + vis1.cols;
    int h = std::max(vis0.rows, vis1.rows);
    cv::Mat out(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    vis0.copyTo(out(cv::Rect(0, 0, vis0.cols, vis0.rows)));
    vis1.copyTo(out(cv::Rect(vis0.cols, 0, vis1.cols, vis1.rows)));
    return out;
}

// Camera-Camera 专用：左右图 + 对极线显示，实时观察对极约束
std::optional<ExtrinsicSE3> run_6dof_interactive_loop_cam_cam(
    ManualExtrinsicAdjuster& adjuster,
    const std::string& window_title,
    const cv::Mat& img_cam0,
    const cv::Mat& img_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1) {

    if (img_cam0.empty() || img_cam1.empty()) {
        UNICALIB_WARN("[ManualCalib] 双相机图像为空，回退到仅 6-DOF 文字窗口");
        return run_6dof_interactive_loop(adjuster, window_title);
    }

    const int total_w = img_cam0.cols + img_cam1.cols;
    const int total_h = std::max(img_cam0.rows, img_cam1.rows);
    const int max_show = 1280;
    int show_w = total_w;
    int show_h = total_h;
    if (show_w > max_show) {
        show_w = max_show;
        show_h = static_cast<int>(std::round(static_cast<double>(total_h) * max_show / total_w));
    }

    cv::namedWindow(window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_title, show_w, show_h);
    UNICALIB_INFO("[ManualCalib][Cam-Cam] 调试日志开关 UNICALIB_MANUAL_CAMCAM_DEBUG={}",
                  manual_camcam_debug_enabled() ? "ON" : "OFF");
    CamCamClickState click_state;
    click_state.img0_w = img_cam0.cols;
    click_state.full_w = total_w;
    click_state.full_h = total_h;
    cv::setMouseCallback(window_title, on_cam_cam_mouse, &click_state);

    bool accepted = false;
    const ExtrinsicSE3 anchor_extrin = adjuster.current();
    EpilineFreezeMode freeze_mode = EpilineFreezeMode::None;
    ManualClickRefiner click_refiner;
    const int min_pairs_for_solve = ManualClickRefiner::Config{}.min_points_cam_cam;
    int successful_click_solves = 0;
    int last_success_pair_count = 0;
    double last_success_rms = -1.0;
    while (true) {
        const ExtrinsicSE3& cur = adjuster.current();
        cv::Mat overlay = render_stereo_epipolar_impl(
            img_cam0, img_cam1, click_state.clicks, intrin0, intrin1, cur, freeze_mode, &anchor_extrin);

        if (overlay.empty()) break;

        Eigen::Vector3d t = cur.POS_TargetInRef;
        Eigen::Vector3d rpy_rad = cur.SO3_TargetInRef.matrix().eulerAngles(0, 1, 2);
        double roll_deg = rpy_rad.x() * 180.0 / M_PI;
        double pitch_deg = rpy_rad.y() * 180.0 / M_PI;
        double yaw_deg = rpy_rad.z() * 180.0 / M_PI;

        int y = 24;
        auto put = [&](const std::string& s, int line = 0) {
            y = 24 + line * 20;
            cv::putText(overlay, s, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        };
        double rms_click = -1.0;
        if (!click_state.clicks.empty()) {
            rms_click = click_refiner.evaluate_cam_cam(click_state.clicks, intrin0, intrin1, cur);
        }
        put("T(m): " + std::to_string(t.x()).substr(0, 7) + " " + std::to_string(t.y()).substr(0, 7) + " " + std::to_string(t.z()).substr(0, 7), 0);
        put("RPY(deg): " + std::to_string(roll_deg).substr(0, 6) + " " + std::to_string(pitch_deg).substr(0, 6) + " " + std::to_string(yaw_deg).substr(0, 6), 1);
        put(std::string("Freeze mode: ") + epiline_freeze_mode_name(freeze_mode) + " (press M to switch)", 2);
        put("Click pairs: " + std::to_string(click_state.clicks.size()) +
            (click_state.has_pending_left ? "  (pending left point)" : "") +
            (rms_click >= 0.0 ? ("  RMS=" + std::to_string(rms_click).substr(0, 7)) : ""), 3);
        put("Mouse: click LEFT image then RIGHT image to form pair", 4);
        put("Q/A Roll  W/S Pitch  E/D Yaw  R/F Tx  T/G Ty  Y/H Tz  U Undo", 5);
        put("Enter=Solve  P=Accept  Z=PopPair  C=ClearPairs  M=FreezeMode  Esc=Cancel", 6);

        cv::imshow(window_title, overlay);

        int key = cv::waitKey(80);
        if (key < 0) continue;

        char k = static_cast<char>(key & 0xFF);
        if (k == 'q') adjuster.apply(AdjustCmd::ROLL_PLUS,   false);
        if (k == 'a') adjuster.apply(AdjustCmd::ROLL_MINUS,  false);
        if (k == 'w') adjuster.apply(AdjustCmd::PITCH_PLUS,  false);
        if (k == 's') adjuster.apply(AdjustCmd::PITCH_MINUS, false);
        if (k == 'e') adjuster.apply(AdjustCmd::YAW_PLUS,    false);
        if (k == 'd') adjuster.apply(AdjustCmd::YAW_MINUS,   false);
        if (k == 'r') adjuster.apply(AdjustCmd::TX_PLUS,     false);
        if (k == 'f') adjuster.apply(AdjustCmd::TX_MINUS,    false);
        if (k == 't') adjuster.apply(AdjustCmd::TY_PLUS,     false);
        if (k == 'g') adjuster.apply(AdjustCmd::TY_MINUS,   false);
        if (k == 'y') adjuster.apply(AdjustCmd::TZ_PLUS,     false);
        if (k == 'h') adjuster.apply(AdjustCmd::TZ_MINUS,    false);
        if (k == 'u') adjuster.undo();
        if (k == 'z' || k == 'Z') {
            if (!click_state.clicks.empty()) click_state.clicks.pop_back();
            click_state.has_pending_left = false;
        }
        if (k == 'c' || k == 'C') {
            click_state.clicks.clear();
            click_state.has_pending_left = false;
        }
        if (k == 'm' || k == 'M') {
            if (freeze_mode == EpilineFreezeMode::None) freeze_mode = EpilineFreezeMode::FreezeLeft;
            else if (freeze_mode == EpilineFreezeMode::FreezeLeft) freeze_mode = EpilineFreezeMode::FreezeRight;
            else freeze_mode = EpilineFreezeMode::None;
            UNICALIB_INFO("[ManualCalib] Cam-Cam 对极线冻结模式切换为 {}", epiline_freeze_mode_name(freeze_mode));
        }
        if (key == 13) {
            if ((int)click_state.clicks.size() >= min_pairs_for_solve) {
                UNICALIB_INFO("[ManualCalib][Cam-Cam] Enter 开始点对求解, 点对数={}", click_state.clicks.size());
                Eigen::Matrix3d R_cur = cur.SO3_TargetInRef.matrix();
                Eigen::Vector3d t_cur = cur.POS_TargetInRef;
                Eigen::Matrix3d t_skew_cur;
                t_skew_cur << 0, -t_cur.z(), t_cur.y(),
                              t_cur.z(), 0, -t_cur.x(),
                              -t_cur.y(), t_cur.x(), 0;
                const Eigen::Matrix3d E_cur = t_skew_cur * R_cur;
                const Eigen::Matrix3d F_cur =
                    intrin1.K().transpose().inverse() * E_cur * intrin0.K().inverse();
                for (size_t i = 0; i < click_state.clicks.size(); ++i) {
                    const auto& c = click_state.clicks[i];
                    Eigen::Vector3d x0_h(c.pt_cam0.x(), c.pt_cam0.y(), 1.0);
                    Eigen::Vector3d x1_h(c.pt_cam1.x(), c.pt_cam1.y(), 1.0);
                    Eigen::Vector3d x0_n((c.pt_cam0.x() - intrin0.cx) / intrin0.fx,
                                         (c.pt_cam0.y() - intrin0.cy) / intrin0.fy, 1.0);
                    Eigen::Vector3d x1_n((c.pt_cam1.x() - intrin1.cx) / intrin1.fx,
                                         (c.pt_cam1.y() - intrin1.cy) / intrin1.fy, 1.0);
                    const double epi_residual = x1_h.transpose() * F_cur * x0_h;
                    UNICALIB_INFO(
                        "[ManualCalib][Cam-Cam] 点对#{:02d} pxL=({:.2f},{:.2f}) pxR=({:.2f},{:.2f}) "
                        "normL=({:.5f},{:.5f},1) normR=({:.5f},{:.5f},1) x1^T F x0={:.6e}",
                        i + 1,
                        c.pt_cam0.x(), c.pt_cam0.y(), c.pt_cam1.x(), c.pt_cam1.y(),
                        x0_n.x(), x0_n.y(), x1_n.x(), x1_n.y(), epi_residual);
                }
                auto solved = click_refiner.refine_cam_cam(
                    img_cam0, img_cam1, intrin0, intrin1, cur, click_state.clicks);
                if (solved.has_value()) {
                    adjuster.set_initial_extrinsic(*solved);
                    successful_click_solves++;
                    last_success_pair_count = static_cast<int>(click_state.clicks.size());
                    last_success_rms = solved->residual_rms;
                    const Eigen::Matrix4d T = solved->SE3_TargetInRef().matrix();
                    UNICALIB_INFO("[ManualCalib][Cam-Cam] 求解结果外参 T_target_in_ref (4x4):");
                    UNICALIB_INFO("[ManualCalib][Cam-Cam] [{: .9f} {: .9f} {: .9f} {: .9f}]",
                                  T(0, 0), T(0, 1), T(0, 2), T(0, 3));
                    UNICALIB_INFO("[ManualCalib][Cam-Cam] [{: .9f} {: .9f} {: .9f} {: .9f}]",
                                  T(1, 0), T(1, 1), T(1, 2), T(1, 3));
                    UNICALIB_INFO("[ManualCalib][Cam-Cam] [{: .9f} {: .9f} {: .9f} {: .9f}]",
                                  T(2, 0), T(2, 1), T(2, 2), T(2, 3));
                    UNICALIB_INFO("[ManualCalib][Cam-Cam] [{: .9f} {: .9f} {: .9f} {: .9f}]",
                                  T(3, 0), T(3, 1), T(3, 2), T(3, 3));
                    UNICALIB_INFO("[ManualCalib] Enter 求解成功: 使用 {} 对点更新外参", click_state.clicks.size());
                } else {
                    UNICALIB_WARN("[ManualCalib] Enter 求解失败: 请检查点对质量或数量");
                }
            } else {
                UNICALIB_WARN("[ManualCalib] 点对不足: {}/{}，无法求解", click_state.clicks.size(), min_pairs_for_solve);
            }
        }
        if (k == 'p' || k == 'P') { accepted = true; break; }
        if (key == 27) { break; }
    }
    if (accepted) {
        UNICALIB_INFO(
            "[ManualCalib][Cam-Cam] 结束并接受: click_solves={} last_pairs={} last_rms={:.6f} final_rpy=({:.4f},{:.4f},{:.4f}) final_t=({:.4f},{:.4f},{:.4f})",
            successful_click_solves, last_success_pair_count, last_success_rms,
            adjuster.current().euler_deg().x(), adjuster.current().euler_deg().y(), adjuster.current().euler_deg().z(),
            adjuster.current().translation().x(), adjuster.current().translation().y(), adjuster.current().translation().z());
    } else {
        UNICALIB_INFO(
            "[ManualCalib][Cam-Cam] 取消退出: click_solves={} current_pairs={} pending_left={}",
            successful_click_solves, click_state.clicks.size(), click_state.has_pending_left);
    }
    cv::setMouseCallback(window_title, nullptr, nullptr);
    cv::destroyWindow(window_title);
    if (accepted) return adjuster.current();
    return std::nullopt;
}
}  // namespace

// ===========================================================================
// ManualExtrinsicAdjuster
// ===========================================================================

void ManualExtrinsicAdjuster::set_initial_extrinsic(const ExtrinsicSE3& extrin) {
    current_ = extrin;
    undo_stack_.clear();
    redo_stack_.clear();
    UNICALIB_INFO("[ManualAdjust] 初始外参:");
    print_current();
}

void ManualExtrinsicAdjuster::set_pose_rpy_deg_xyz_m(
    double roll_deg, double pitch_deg, double yaw_deg,
    double tx_m, double ty_m, double tz_m) {

    if (cfg_.enable_undo) {
        if ((int)undo_stack_.size() >= cfg_.max_undo_depth)
            undo_stack_.pop_front();
        undo_stack_.push_back(current_);
        redo_stack_.clear();
    }

    const double deg2rad = M_PI / 180.0;
    Eigen::AngleAxisd rx(roll_deg * deg2rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch_deg * deg2rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw_deg * deg2rad, Eigen::Vector3d::UnitZ());
    Sophus::SO3d R(rz.toRotationMatrix() * ry.toRotationMatrix() * rx.toRotationMatrix());
    current_.SO3_TargetInRef = R;
    current_.POS_TargetInRef = Eigen::Vector3d(tx_m, ty_m, tz_m);

    UNICALIB_INFO("[ManualAdjust] 外参已设为面板指定 RPY(deg)+XYZ(m)");
    print_current();
}

void ManualExtrinsicAdjuster::apply(AdjustCmd cmd, bool fast_mode) {
    double rot_step = fast_mode ? cfg_.step.rot_fast_deg  : cfg_.step.rot_step_deg;
    double t_step   = fast_mode ? cfg_.step.trans_fast_m  : cfg_.step.trans_step_m;

    switch (cmd) {
        case AdjustCmd::ROLL_PLUS:   apply_delta_rotation(+rot_step, 0, 0); break;
        case AdjustCmd::ROLL_MINUS:  apply_delta_rotation(-rot_step, 0, 0); break;
        case AdjustCmd::PITCH_PLUS:  apply_delta_rotation(0, +rot_step, 0); break;
        case AdjustCmd::PITCH_MINUS: apply_delta_rotation(0, -rot_step, 0); break;
        case AdjustCmd::YAW_PLUS:    apply_delta_rotation(0, 0, +rot_step); break;
        case AdjustCmd::YAW_MINUS:   apply_delta_rotation(0, 0, -rot_step); break;
        case AdjustCmd::TX_PLUS:     apply_delta_translation(+t_step, 0, 0); break;
        case AdjustCmd::TX_MINUS:    apply_delta_translation(-t_step, 0, 0); break;
        case AdjustCmd::TY_PLUS:     apply_delta_translation(0, +t_step, 0); break;
        case AdjustCmd::TY_MINUS:    apply_delta_translation(0, -t_step, 0); break;
        case AdjustCmd::TZ_PLUS:     apply_delta_translation(0, 0, +t_step); break;
        case AdjustCmd::TZ_MINUS:    apply_delta_translation(0, 0, -t_step); break;
        case AdjustCmd::UNDO:  undo(); break;
        default: break;
    }
}

void ManualExtrinsicAdjuster::apply_delta_rotation(
    double roll_deg, double pitch_deg, double yaw_deg) {

    // 保存到撤销栈
    if (cfg_.enable_undo) {
        if ((int)undo_stack_.size() >= cfg_.max_undo_depth)
            undo_stack_.pop_front();
        undo_stack_.push_back(current_);
        redo_stack_.clear();
    }

    // 右乘增量旋转 (RPY 顺序: R(z)*R(y)*R(x))
    const double deg2rad = M_PI / 180.0;
    Eigen::AngleAxisd rx(roll_deg  * deg2rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch_deg * deg2rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw_deg   * deg2rad, Eigen::Vector3d::UnitZ());
    Sophus::SO3d delta_rot(rz.toRotationMatrix() *
                           ry.toRotationMatrix() *
                           rx.toRotationMatrix());

    // T_new = T_old * delta (右乘 — 在 target 坐标系下调整)
    auto T_old = current_.SE3_TargetInRef();
    Sophus::SE3d delta(delta_rot, Eigen::Vector3d::Zero());
    current_.set_SE3(T_old * delta);

    UNICALIB_DEBUG("[ManualAdjust] 旋转增量 RPY=({:.3f}, {:.3f}, {:.3f})deg",
                   roll_deg, pitch_deg, yaw_deg);
    print_current();
}

void ManualExtrinsicAdjuster::apply_delta_translation(
    double dx_m, double dy_m, double dz_m) {

    if (cfg_.enable_undo) {
        if ((int)undo_stack_.size() >= cfg_.max_undo_depth)
            undo_stack_.pop_front();
        undo_stack_.push_back(current_);
        redo_stack_.clear();
    }

    current_.POS_TargetInRef += Eigen::Vector3d(dx_m, dy_m, dz_m);

    UNICALIB_DEBUG("[ManualAdjust] 平移增量 dxyz=({:.4f}, {:.4f}, {:.4f})m",
                   dx_m, dy_m, dz_m);
    print_current();
}

bool ManualExtrinsicAdjuster::undo() {
    if (undo_stack_.empty()) {
        UNICALIB_WARN("[ManualAdjust] 无可撤销操作");
        return false;
    }
    redo_stack_.push_back(current_);
    current_ = undo_stack_.back();
    undo_stack_.pop_back();
    UNICALIB_INFO("[ManualAdjust] 撤销 (undo_stack 剩余: {})", undo_stack_.size());
    print_current();
    return true;
}

bool ManualExtrinsicAdjuster::redo() {
    if (redo_stack_.empty()) {
        UNICALIB_WARN("[ManualAdjust] 无可重做操作");
        return false;
    }
    undo_stack_.push_back(current_);
    current_ = redo_stack_.back();
    redo_stack_.pop_back();
    UNICALIB_INFO("[ManualAdjust] 重做 (redo_stack 剩余: {})", redo_stack_.size());
    print_current();
    return true;
}

void ManualExtrinsicAdjuster::print_current() const {
    auto euler = current_.euler_deg();
    auto t     = current_.translation();
    UNICALIB_INFO("[ManualAdjust] 当前外参 [{}→{}]:",
                  current_.ref_sensor_id, current_.target_sensor_id);
    UNICALIB_INFO("  旋转 RPY: ({:.4f}, {:.4f}, {:.4f}) deg",
                  euler.x(), euler.y(), euler.z());
    UNICALIB_INFO("  平移 XYZ: ({:.4f}, {:.4f}, {:.4f}) m",
                  t.x(), t.y(), t.z());
}

void ManualExtrinsicAdjuster::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_ERROR("[ManualAdjust] 无法写入: {}", path);
        return;
    }
    auto euler = current_.euler_deg();
    auto t     = current_.translation();
    f << "# UniCalib 手动校准结果\n";
    f << "ref_sensor_id: " << current_.ref_sensor_id << "\n";
    f << "target_sensor_id: " << current_.target_sensor_id << "\n";
    f << "rotation_rpy_deg: [" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]\n";
    f << "translation_xyz_m: [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
    f << "time_offset_s: " << current_.time_offset_s << "\n";
    f << "residual_rms: " << current_.residual_rms << "\n";
    UNICALIB_INFO("[ManualAdjust] 结果已保存: {}", path);
}

// ===========================================================================
// ManualClickRefiner
// ===========================================================================

std::optional<ExtrinsicSE3> ManualClickRefiner::refine_lidar_cam(
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& init_extrin,
    const std::vector<ClickPoint2D3D>& user_clicks) {

    UNICALIB_INFO("[ClickRefine] LiDAR-Camera 手动点击精化");
    UNICALIB_INFO("[ClickRefine] 点击点数量: {}", user_clicks.size());

    if (user_clicks.empty()) {
        UNICALIB_WARN("[ClickRefine] 未提供用户点击点, 返回初始外参");
        return init_extrin;
    }

    // 过滤有 3D 坐标的点击
    std::vector<ClickPoint2D3D> valid_clicks;
    for (const auto& c : user_clicks) {
        if (c.has_3d) valid_clicks.push_back(c);
    }

    if ((int)valid_clicks.size() < cfg_.min_points_lidar_cam) {
        UNICALIB_WARN("[ClickRefine] 有效3D-2D对应点不足 ({}/{})",
                      valid_clicks.size(), cfg_.min_points_lidar_cam);
        return std::nullopt;
    }

    return optimize_from_clicks(valid_clicks, cam_intrin, init_extrin,
                                 "lidar_cam_click");
}

std::optional<ExtrinsicSE3> ManualClickRefiner::refine_cam_cam(
    const cv::Mat& img_cam0,
    const cv::Mat& img_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& init_extrin,
    const std::vector<ClickCorrespondence>& user_clicks) {

    UNICALIB_INFO("[ClickRefine] Camera-Camera 手动点击精化");
    UNICALIB_INFO("[ClickRefine] 对应点数量: {}", user_clicks.size());

    if ((int)user_clicks.size() < cfg_.min_points_cam_cam) {
        UNICALIB_WARN("[ClickRefine] 对应点不足 ({}/{})",
                      user_clicks.size(), cfg_.min_points_cam_cam);
        return std::nullopt;
    }

    return optimize_cam_cam_from_clicks(user_clicks, intrin0, intrin1,
                                         init_extrin, "cam_cam_click");
}

double ManualClickRefiner::evaluate_lidar_cam(
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& extrin) const {
    (void)scan; (void)image;
    UNICALIB_TRACE("[ClickRefine] LiDAR-Cam 评估 (占位 — 需点云投影实现)");
    return extrin.residual_rms;
}

double ManualClickRefiner::evaluate_cam_cam(
    const std::vector<ClickCorrespondence>& clicks,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin) const {
    if (clicks.empty()) return 0.0;

    auto T = extrin.SE3_TargetInRef();
    double sum_sq = 0.0;
    int cnt = 0;

    for (const auto& c : clicks) {
        // 将 cam0 点归一化后通过 T 投影到 cam1
        Eigen::Vector3d p0_norm(
            (c.pt_cam0.x() - intrin0.cx) / intrin0.fx,
            (c.pt_cam0.y() - intrin0.cy) / intrin0.fy,
            1.0);
        Eigen::Vector3d p1_cam = T.rotationMatrix() * p0_norm + T.translation();
        if (p1_cam.z() < 1e-6) continue;
        double u = intrin1.fx * p1_cam.x() / p1_cam.z() + intrin1.cx;
        double v = intrin1.fy * p1_cam.y() / p1_cam.z() + intrin1.cy;
        double du = u - c.pt_cam1.x();
        double dv = v - c.pt_cam1.y();
        sum_sq += du*du + dv*dv;
        cnt++;
    }
    return cnt > 0 ? std::sqrt(sum_sq / cnt) : 0.0;
}

cv::Mat ManualClickRefiner::render_stereo_epipolar(
    const cv::Mat& img0,
    const cv::Mat& img1,
    const std::vector<ClickCorrespondence>& clicks,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin) const {
    return render_stereo_epipolar_impl(img0, img1, clicks, intrin0, intrin1, extrin);
}

std::optional<ExtrinsicSE3> ManualClickRefiner::optimize_from_clicks(
    const std::vector<ClickPoint2D3D>& clicks,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& init_extrin,
    const std::string& log_prefix) {

    if (clicks.empty()) {
        UNICALIB_WARN("[ClickOpt-{}] 点击点为空，无法优化", log_prefix);
        return std::nullopt;
    }
    UNICALIB_INFO("[ClickOpt-{}] Ceres 优化 (点数={})", log_prefix, clicks.size());

    // 初始化参数: angle-axis + translation
    auto T_init = init_extrin.SE3_TargetInRef();
    Eigen::AngleAxisd aa(T_init.rotationMatrix());
    double rot[3]   = {aa.axis().x() * aa.angle(),
                       aa.axis().y() * aa.angle(),
                       aa.axis().z() * aa.angle()};
    double trans[3] = {T_init.translation().x(),
                       T_init.translation().y(),
                       T_init.translation().z()};

    ceres::Problem problem;
    for (const auto& c : clicks) {
        auto* cost = LidarCamReprojCost::Create(
            c.world_pt, c.img_pt,
            cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy);
        problem.AddResidualBlock(cost,
            new ceres::HuberLoss(cfg_.ransac_thresh_px),
            rot, trans);
    }

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = cfg_.ceres_max_iter;
    opts.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    UNICALIB_INFO("[ClickOpt-{}] 优化结果: {}", log_prefix,
                  summary.BriefReport());
    UNICALIB_INFO("[ClickOpt-{}] 最终代价: {:.6f} → {:.6f}",
                  log_prefix, summary.initial_cost, summary.final_cost);
    UNICALIB_CALC("手动标定 LiDAR-Cam 点击优化 initial_cost={:.6f} final_cost={:.6f} iter={} converged={}",
                  summary.initial_cost, summary.final_cost, static_cast<int>(summary.iterations.size()),
                  summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");

    if (!summary.IsSolutionUsable()) {
        UNICALIB_WARN("[ClickOpt-{}] 优化未收敛", log_prefix);
        return std::nullopt;
    }

    // 构建结果
    ExtrinsicSE3 result = init_extrin;
    Eigen::Vector3d rv(rot[0], rot[1], rot[2]);
    result.SO3_TargetInRef = (rv.norm() < 1e-10)
        ? Sophus::SO3d()
        : Sophus::SO3d(Eigen::AngleAxisd(rv.norm(), rv.normalized()).toRotationMatrix());
    result.POS_TargetInRef = Eigen::Vector3d(trans[0], trans[1], trans[2]);
    result.residual_rms = std::sqrt(summary.final_cost / clicks.size());
    result.is_converged = true;

    UNICALIB_CALC("手动标定 LiDAR-Cam 点击优化完成 点数={} residual_rms={:.4f}px", clicks.size(), result.residual_rms);
    UNICALIB_INFO("[ClickOpt-{}] 结果 RMS: {:.4f} px", log_prefix, result.residual_rms);
    return result;
}

std::optional<ExtrinsicSE3> ManualClickRefiner::optimize_cam_cam_from_clicks(
    const std::vector<ClickCorrespondence>& clicks,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& init_extrin,
    const std::string& log_prefix) {

    if (clicks.empty()) {
        UNICALIB_WARN("[ClickOpt-{}] 对应点为空，无法优化", log_prefix);
        return std::nullopt;
    }
    UNICALIB_INFO("[ClickOpt-{}] Cam-Cam 对极优化 (点数={})",
                  log_prefix, clicks.size());

    auto T_init = init_extrin.SE3_TargetInRef();
    Eigen::AngleAxisd aa(T_init.rotationMatrix());
    double rot[3]   = {aa.axis().x() * aa.angle(),
                       aa.axis().y() * aa.angle(),
                       aa.axis().z() * aa.angle()};
    double trans[3] = {T_init.translation().x(),
                       T_init.translation().y(),
                       T_init.translation().z()};

    ceres::Problem problem;
    for (const auto& c : clicks) {
        Eigen::Vector2d p0_norm(
            (c.pt_cam0.x() - intrin0.cx) / intrin0.fx,
            (c.pt_cam0.y() - intrin0.cy) / intrin0.fy);
        Eigen::Vector2d p1_norm(
            (c.pt_cam1.x() - intrin1.cx) / intrin1.fx,
            (c.pt_cam1.y() - intrin1.cy) / intrin1.fy);

        auto* cost = CamCamEpipolarCost::Create(p0_norm, p1_norm);
        problem.AddResidualBlock(cost,
            new ceres::HuberLoss(1.0 / std::max(intrin0.fx, intrin1.fx)),
            rot, trans);
    }

    // 固定平移的尺度 (对极约束 t 只有方向, 不含尺度)
    problem.SetManifold(trans, new ceres::SphereManifold<3>());

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = cfg_.ceres_max_iter;
    opts.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    UNICALIB_INFO("[ClickOpt-{}] 最终代价: {:.6f} → {:.6f}",
                  log_prefix, summary.initial_cost, summary.final_cost);
    UNICALIB_CALC("手动标定 Cam-Cam 对极优化 initial_cost={:.6f} final_cost={:.6f} iter={} 点数={} converged={}",
                  summary.initial_cost, summary.final_cost, static_cast<int>(summary.iterations.size()),
                  clicks.size(), summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");

    if (!summary.IsSolutionUsable()) {
        UNICALIB_WARN("[ClickOpt-{}] 优化未收敛", log_prefix);
        return std::nullopt;
    }

    ExtrinsicSE3 result = init_extrin;
    double angle = std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);
    if (angle > 1e-10) {
        Eigen::AngleAxisd aa_result(angle,
            Eigen::Vector3d(rot[0], rot[1], rot[2]) / angle);
        result.SO3_TargetInRef = Sophus::SO3d(aa_result.toRotationMatrix());
    }
    // 注意: Cam-Cam 对极约束无法恢复平移尺度, 保留 init 的平移
    result.residual_rms = std::sqrt(summary.final_cost / clicks.size());
    result.is_converged = true;

    UNICALIB_CALC("手动标定 Cam-Cam 对极优化完成 点数={} residual_rms={:.4f}", clicks.size(), result.residual_rms);
    UNICALIB_INFO("[ClickOpt-{}] 结果 RMS: {:.6f}", log_prefix, result.residual_rms);
    return result;
}

void ManualClickRefiner::save_clicks(
    const std::string& path,
    const std::vector<ClickCorrespondence>& clicks) const {
    std::ofstream f(path);
    f << "# UniCalib 手动点击对应点\n";
    f << "num_points: " << clicks.size() << "\n";
    f << "points:\n";
    for (size_t i = 0; i < clicks.size(); ++i) {
        f << "  - id: " << i << "\n";
        f << "    pt_cam0: [" << clicks[i].pt_cam0.x() << ", "
          << clicks[i].pt_cam0.y() << "]\n";
        f << "    pt_cam1: [" << clicks[i].pt_cam1.x() << ", "
          << clicks[i].pt_cam1.y() << "]\n";
        f << "    confidence: " << clicks[i].confidence << "\n";
    }
    UNICALIB_INFO("[ClickRefine] 点击点已保存: {} ({} 点)", path, clicks.size());
}

// ===========================================================================
// ManualCalibSession
// ===========================================================================

ManualCalibSession::ManualCalibSession()
    : ManualCalibSession(SessionConfig{}) {}

ManualCalibSession::ManualCalibSession(const SessionConfig& cfg)
    : cfg_(cfg) {
    if (cfg_.session_id.empty()) {
        session_id_ = "session_" + ts_str();
        std::replace(session_id_.begin(), session_id_.end(), ' ', '_');
        std::replace(session_id_.begin(), session_id_.end(), ':', '-');
    } else {
        session_id_ = cfg_.session_id;
    }
    fs::create_directories(cfg_.save_dir);
    UNICALIB_INFO("[ManualSession] 会话ID: {}", session_id_);
    UNICALIB_INFO("[ManualSession] 保存目录: {}", cfg_.save_dir);
}

ExtrinsicSE3 ManualCalibSession::run_lidar_cam(
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& auto_result,
    double auto_rms) {

    UNICALIB_INFO("[ManualSession] 启动 LiDAR-Camera 手动校准");
    UNICALIB_INFO("[ManualSession] 自动标定 RMS: {:.4f} px (阈值建议 < 2.0px)",
                  auto_rms);
    log_session_event("lidar_cam_start",
                       "auto_rms=" + std::to_string(auto_rms));

    // 初始化调整器
    ManualExtrinsicAdjuster::Config adj_cfg;
    adjuster_.set_initial_extrinsic(auto_result);

    // 操作说明
    UNICALIB_INFO("[ManualSession] ===== 手动校准操作说明 =====");
    UNICALIB_INFO("  Q/A: Roll ±{:.1f}deg   W/S: Pitch ±{:.1f}deg   E/D: Yaw ±{:.1f}deg",
                  adj_cfg.step.rot_step_deg, adj_cfg.step.rot_step_deg,
                  adj_cfg.step.rot_step_deg);
    UNICALIB_INFO("  R/F: Tx ±{:.1f}cm   T/G: Ty ±{:.1f}cm   Y/H: Tz ±{:.1f}cm",
                  adj_cfg.step.trans_step_m*100, adj_cfg.step.trans_step_m*100,
                  adj_cfg.step.trans_step_m*100);
    UNICALIB_INFO("  Shift+键: 10倍步长");
    UNICALIB_INFO("  P: 进入点击精化模式  U: 撤销  Z: 重做");
    UNICALIB_INFO("  S: 保存  ESC: 接受并退出");
    UNICALIB_INFO("  点云投影质量越高 (边缘对齐越好) 表示标定越准确");
    ExtrinsicSE3 result = auto_result;
    result.residual_rms = auto_rms;
    if (cfg_.enable_interactive_gui) {
        std::optional<ExtrinsicSE3> adjusted;
#if UNICALIB_WITH_PANGOLIN
        if (cfg_.use_pangolin_manual_panel) {
            ManualAdjustStep step;
            adjusted = run_lidar_cam_pangolin_panel(
                adjuster_, scan, image, cam_intrin, step, &auto_result);
        } else
#endif
        {
            adjusted = run_6dof_interactive_loop_lidar_cam(
                adjuster_, "LiDAR-Cam Manual Adjust", scan, image, cam_intrin);
        }
        if (adjusted.has_value()) {
            result = *adjusted;
            if (!(result.residual_rms > 0.0)) result.residual_rms = auto_rms;
            UNICALIB_INFO("[ManualSession] 用户接受手动调整结果");
            UNICALIB_INFO("[ManualSession] Cam-Cam 最终采用外参 RMS={:.6f} (manual if available, else auto={:.6f})",
                         result.residual_rms, auto_rms);
        } else {
            UNICALIB_INFO("[ManualSession] 用户取消，保留自动标定结果");
        }
    } else {
        UNICALIB_INFO("[ManualSession] 未启用交互 GUI (enable_interactive_gui=false)，返回自动标定结果");
    }

    log_session_event("lidar_cam_end",
        "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["lidar_cam"] = result;

    if (cfg_.auto_save) {
        save_session();
    }
    return result;
}

ExtrinsicSE3 ManualCalibSession::run_cam_cam(
    const cv::Mat& img_cam0,
    const cv::Mat& img_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& auto_result,
    double auto_rms) {

    UNICALIB_INFO("[ManualSession] 启动 Camera-Camera 手动校准");
    UNICALIB_INFO("[ManualSession] 自动标定 RMS: {:.4f} px", auto_rms);
    log_session_event("cam_cam_start",
                       "auto_rms=" + std::to_string(auto_rms));

    adjuster_.set_initial_extrinsic(auto_result);

    UNICALIB_INFO("[ManualSession] 双目相机手动校准模式");
    UNICALIB_INFO("[ManualSession] 对极线可视化质量越好表示标定越准确");

    ExtrinsicSE3 result = auto_result;
    result.residual_rms = auto_rms;
    if (cfg_.enable_interactive_gui) {
        auto adjusted = run_6dof_interactive_loop_cam_cam(
            adjuster_, "Cam-Cam Manual Adjust",
            img_cam0, img_cam1, intrin0, intrin1);
        if (adjusted.has_value()) {
            result = *adjusted;
            result.residual_rms = auto_rms;
            UNICALIB_INFO("[ManualSession] 用户接受手动调整结果");
        } else {
            UNICALIB_INFO("[ManualSession] 用户取消，保留自动标定结果");
        }
    }

    log_session_event("cam_cam_end",
        "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["cam_cam"] = result;

    if (cfg_.auto_save) save_session();
    return result;
}

ExtrinsicSE3 ManualCalibSession::run_imu_lidar(
    const std::vector<LiDARScan>& scans,
    const std::vector<IMUFrame>& imu_data,
    const ExtrinsicSE3& auto_result,
    double auto_rms) {

    UNICALIB_INFO("[ManualSession] 启动 IMU-LiDAR 手动校准");
    UNICALIB_INFO("[ManualSession] 自动标定旋转残差: {:.4f} deg", auto_rms);
    UNICALIB_INFO("[ManualSession] IMU-LiDAR 手动校准通过旋转增量验证");
    UNICALIB_INFO("[ManualSession] 可视化: 将 IMU 积分轨迹与 LiDAR 里程计对比");
    log_session_event("imu_lidar_start",
                       "auto_rms=" + std::to_string(auto_rms));

    adjuster_.set_initial_extrinsic(auto_result);
    ExtrinsicSE3 result = auto_result;

    log_session_event("imu_lidar_end",
        "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["imu_lidar"] = result;

    if (cfg_.auto_save) save_session();
    return result;
}

ExtrinsicSE3 ManualCalibSession::run_lidar_lidar(
    const ExtrinsicSE3& auto_result,
    double auto_fitness_or_rms,
    const LiDARScan* ref_first,
    const LiDARScan* target_first) {

    UNICALIB_INFO("[ManualSession] 启动 LiDAR-LiDAR 手动校准 cfg.enable_interactive_gui={}",
                  cfg_.enable_interactive_gui);
    if (ref_first && target_first) {
        const size_t ref_n = (ref_first->cloud ? ref_first->cloud->size() : 0);
        const size_t tgt_n = (target_first->cloud ? target_first->cloud->size() : 0);
        UNICALIB_INFO("[Manual-Refine] LiDAR-LiDAR 手动调整使用一对点云（通常为时间对齐或调用方选定的 ref/target 帧） ref_points={} target_points={}",
                      ref_n, tgt_n);
    } else {
        UNICALIB_WARN("[Manual-Refine] 未提供首帧点云，可能回退到无点云 6DOF 文本/2D 模式");
    }
    UNICALIB_INFO("[ManualSession] 自动标定 fitness/rms: {:.4f}", auto_fitness_or_rms);
    log_session_event("lidar_lidar_start",
                      "auto_fitness_or_rms=" + std::to_string(auto_fitness_or_rms));

    adjuster_.set_initial_extrinsic(auto_result);
    ExtrinsicSE3 result = auto_result;
    result.residual_rms = (auto_fitness_or_rms >= 0 && auto_fitness_or_rms <= 1)
        ? (1.0 - auto_fitness_or_rms) : auto_fitness_or_rms;

    if (cfg_.enable_interactive_gui) {
        std::optional<ExtrinsicSE3> adjusted;
#if UNICALIB_WITH_PANGOLIN
        if (ref_first && target_first && ref_first->cloud && !ref_first->cloud->empty() &&
            target_first->cloud && !target_first->cloud->empty()) {
            UNICALIB_INFO("[ManualSession] 路径选择: run_lidar_lidar_pangolin_panel (UNICALIB_WITH_PANGOLIN=true)");
            ManualAdjustStep step;
            adjusted = run_lidar_lidar_pangolin_panel(
                adjuster_, *ref_first, *target_first, step, &auto_result);
        } else
#endif
        {
            if (ref_first && target_first) {
                UNICALIB_INFO("[ManualSession] 路径选择: run_6dof_interactive_loop_lidar_lidar (有点云但未用 Pangolin 面板)");
                adjusted = run_6dof_interactive_loop_lidar_lidar(
                    adjuster_, "LiDAR-LiDAR Manual Adjust", *ref_first, *target_first);
            } else {
                UNICALIB_INFO("[ManualSession] 路径选择: run_6dof_interactive_loop (无点云)");
                adjusted = run_6dof_interactive_loop(adjuster_, "LiDAR-LiDAR Manual Adjust");
            }
        }
        if (adjusted.has_value()) {
            result = *adjusted;
            UNICALIB_INFO("[ManualSession] 用户接受 LiDAR-LiDAR 手动调整结果");
        } else {
            UNICALIB_INFO("[ManualSession] 用户取消，保留自动标定结果");
        }
    }

    log_session_event("lidar_lidar_end",
                      "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["lidar_lidar"] = result;
    if (cfg_.auto_save) save_session();
    return result;
}

void ManualCalibSession::log_session_event(const std::string& event,
                                            const std::string& detail) {
    SessionEvent ev;
    ev.timestamp = ts_str();
    ev.event = event;
    ev.detail = detail;
    ev.extrin_snapshot = adjuster_.current();
    history_.push_back(ev);

    if (cfg_.log_all_steps) {
        UNICALIB_DEBUG("[ManualSession] 事件: {} | {}", event, detail);
    }
}

void ManualCalibSession::save_session(const std::string& path) const {
    std::string save_path = path.empty() ?
        cfg_.save_dir + "/" + session_id_ + ".yaml" : path;

    std::ofstream f(save_path);
    if (!f.is_open()) {
        UNICALIB_WARN("[ManualSession] 无法保存: {}", save_path);
        return;
    }
    f << "session_id: " << session_id_ << "\n";
    f << "events:\n";
    for (const auto& ev : history_) {
        f << "  - ts: " << ev.timestamp << "\n";
        f << "    event: " << ev.event << "\n";
        f << "    detail: \"" << ev.detail << "\"\n";
    }
    f << "final_extrinsics:\n";
    for (const auto& [key, extrin] : final_extrinsics_) {
        auto euler = extrin.euler_deg();
        auto t     = extrin.translation();
        f << "  " << key << ":\n";
        f << "    ref: " << extrin.ref_sensor_id << "\n";
        f << "    target: " << extrin.target_sensor_id << "\n";
        f << "    rpy_deg: [" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]\n";
        f << "    xyz_m: [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
        f << "    rms: " << extrin.residual_rms << "\n";
    }
    UNICALIB_INFO("[ManualSession] 会话已保存: {}", save_path);
}

void ManualCalibSession::export_results(CalibParamManager& pm) const {
    for (const auto& [key, extrin] : final_extrinsics_) {
        auto ptr = pm.get_or_create_extrinsic(
            extrin.ref_sensor_id, extrin.target_sensor_id);
        *ptr = extrin;
        UNICALIB_INFO("[ManualSession] 导出外参 [{}→{}] RMS={:.4f}",
                      extrin.ref_sensor_id, extrin.target_sensor_id,
                      extrin.residual_rms);
    }
}

}  // namespace ns_unicalib
