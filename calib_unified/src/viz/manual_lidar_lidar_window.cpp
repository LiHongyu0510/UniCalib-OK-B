/**
 * UniCalib — LiDAR-LiDAR 手动标定 Pangolin 3D 窗口实现
 * 参考 SensorsCalibration lidar2lidar/manual_calib。
 */

#if UNICALIB_WITH_PANGOLIN

#include "unicalib/viz/manual_lidar_lidar_window.h"
#include "unicalib/viz/manual_pangolin_ui_scale.h"
#include "unicalib/common/logger.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/viz/pangolin_compat.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <pangolin/utils/params.h>
#include <pangolin/var/var.h>
#include <pangolin/gl/gl.h>
#include <Eigen/Core>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>

namespace ns_unicalib {

namespace {

struct RGB {
    unsigned char r, g, b;
};

RGB intensityToColor(uint8_t val) {
    int v = static_cast<int>(val);
    int r, g, b;
    if (v < 128) r = 0;
    else if (v < 192) r = 255 * (v - 128) / 64;
    else r = 255;
    if (v < 64) g = 255 * v / 64;
    else if (v < 192) g = 255;
    else g = -255 * (v - 192) / 63 + 255;
    if (v < 64) b = 255;
    else if (v < 128) b = -255 * (v - 64) / 63 + 255;
    else b = 0;
    return {static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b)};
}

bool kbhit() {
    termios term;
    tcgetattr(0, &term);
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    tcsetattr(0, TCSANOW, &term);
    return byteswaiting > 0;
}

// Ref 点云：Ref 系下不变换，上传到 GL buffer（灰或强度着色）
void uploadRefCloud(const LiDARScan& ref_scan, bool intensity_color, bool solid_ref_tgt,
                    std::vector<float>* vertices, std::vector<unsigned char>* colors) {
    if (!ref_scan.cloud || ref_scan.cloud->empty()) return;
    const auto& cloud = *ref_scan.cloud;
    vertices->resize(cloud.size() * 3);
    colors->resize(cloud.size() * 3);
    for (size_t i = 0; i < cloud.size(); ++i) {
        (*vertices)[i * 3 + 0] = cloud.points[i].x;
        (*vertices)[i * 3 + 1] = cloud.points[i].y;
        (*vertices)[i * 3 + 2] = cloud.points[i].z;
        uint8_t I = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, cloud.points[i].intensity)));
        if (solid_ref_tgt) {
            // 高对比：青色系 Ref，与 Target 橙红分离（不受强度着色干扰）
            (*colors)[i * 3 + 0] = 30;
            (*colors)[i * 3 + 1] = 255;
            (*colors)[i * 3 + 2] = 200;
        } else if (intensity_color) {
            RGB c = intensityToColor(I);
            (*colors)[i * 3 + 0] = c.r;
            (*colors)[i * 3 + 1] = c.g;
            (*colors)[i * 3 + 2] = c.b;
        } else {
            (*colors)[i * 3 + 0] = (*colors)[i * 3 + 1] = (*colors)[i * 3 + 2] = I;
        }
    }
}

// Target 点云：用 T_TargetInRef 变换到 Ref 系后上传
void uploadTargetCloud(const LiDARScan& target_scan, const Eigen::Matrix4d& T_TargetInRef,
                      bool intensity_color, bool solid_ref_tgt,
                      std::vector<float>* vertices, std::vector<unsigned char>* colors) {
    if (!target_scan.cloud || target_scan.cloud->empty()) return;
    const auto& cloud = *target_scan.cloud;
    vertices->resize(cloud.size() * 3);
    colors->resize(cloud.size() * 3);
    for (size_t i = 0; i < cloud.size(); ++i) {
        Eigen::Vector4d p(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1.0);
        Eigen::Vector4d q = T_TargetInRef * p;
        (*vertices)[i * 3 + 0] = static_cast<float>(q.x());
        (*vertices)[i * 3 + 1] = static_cast<float>(q.y());
        (*vertices)[i * 3 + 2] = static_cast<float>(q.z());
        uint8_t I = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, cloud.points[i].intensity)));
        if (solid_ref_tgt) {
            (*colors)[i * 3 + 0] = 255;
            (*colors)[i * 3 + 1] = 85;
            (*colors)[i * 3 + 2] = 0;
        } else if (intensity_color) {
            RGB c = intensityToColor(I);
            (*colors)[i * 3 + 0] = c.r;
            (*colors)[i * 3 + 1] = c.g;
            (*colors)[i * 3 + 2] = c.b;
        } else {
            (*colors)[i * 3 + 0] = 255;
            (*colors)[i * 3 + 1] = (*colors)[i * 3 + 2] = 0;
        }
    }
}

}  // namespace

std::optional<ExtrinsicSE3> run_lidar_lidar_pangolin_panel(
    ManualExtrinsicAdjuster& adjuster,
    const LiDARScan& ref_scan,
    const LiDARScan& target_scan,
    const ManualAdjustStep& step,
    const ExtrinsicSE3* initial_extrinsic) {

    if (!ref_scan.cloud || ref_scan.cloud->empty() || !target_scan.cloud || target_scan.cloud->empty()) {
        UNICALIB_WARN("[ManualLidarLidar] Ref 或 Target 点云为空");
        return std::nullopt;
    }
    UNICALIB_INFO("[ManualLidarLidar] 进入面板函数: ref_points={} target_points={} step(rot_deg={}, trans_m={})",
                  ref_scan.cloud->size(), target_scan.cloud->size(), step.rot_step_deg, step.trans_step_m);

    // 左栏：栏宽 + Pangolin 默认字体放大（控件行高随字号变化，非仅拉伸布局）
    const int panel_w = kManualPangolinPanelWidthPx;
    const int width = 1920 + (panel_w - 180);  // 保持右侧 3D 视口约 1740px 宽
    const int height = 1080;
    const int view_3d_w = width - panel_w;

    UNICALIB_INFO("[ManualLidarLidar] CreateWindowAndBind 开始 title='LiDAR-LiDAR Manual Calibration' size={}x{} panel_w={} font_px={}",
                  width, height, panel_w, kManualPangolinDefaultFontPx);
    pangolin::Params win_params;
    win_params.Set("default_font_size", kManualPangolinDefaultFontPx);
    pangolin::CreateWindowAndBind("LiDAR-LiDAR Manual Calibration", width, height, win_params);
    UNICALIB_INFO("[ManualLidarLidar] CreateWindowAndBind 完成 ShouldQuit={}", pangolin::ShouldQuit());
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // 与右侧 3D 子视口一致的投影与宽高比（原先用全窗口 width 与 1024x768 投影，会严重拉伸/错位）
    Eigen::Vector3d pmin(1e30, 1e30, 1e30), pmax(-1e30, -1e30, -1e30);
    auto expand_xyz = [&](const Eigen::Vector3d& v) {
        pmin = pmin.cwiseMin(v);
        pmax = pmax.cwiseMax(v);
    };
    for (const auto& pt : ref_scan.cloud->points)
        expand_xyz(Eigen::Vector3d(pt.x, pt.y, pt.z));
    {
        Eigen::Matrix4d Ttr = adjuster.current().SE3_TargetInRef().matrix();
        for (const auto& pt : target_scan.cloud->points) {
            Eigen::Vector4d q = Ttr * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
            expand_xyz(q.head<3>());
        }
    }
    Eigen::Vector3d center = 0.5 * (pmin + pmax);
    double diag = (pmax - pmin).norm();
    if (!std::isfinite(diag) || diag < 1e-6) {
        center.setZero();
        diag = 80.0;
    }
    const double dist = std::max(8.0, std::min(5000.0, diag * 0.65));
    const double z_far = std::max(2000.0, diag * 15.0);
    const double fx = 0.4 * static_cast<double>(std::max(view_3d_w, height));
    const double cx = 0.5 * static_cast<double>(view_3d_w);
    const double cy = 0.5 * static_cast<double>(height);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(view_3d_w, height, fx, fx, cx, cy, 0.05, z_far),
        pangolin::ModelViewLookAt(
            center.x() + dist * 0.65, center.y() + dist * 0.55, center.z() + dist * 0.45,
            center.x(), center.y(), center.z(),
            0.0, 0.0, 1.0));
    UNICALIB_INFO("[ManualLidarLidar] 相机初始化 center=({}, {}, {}) diag={} dist={} z_far={} fx={}",
                  center.x(), center.y(), center.z(), diag, dist, z_far, fx);

    pangolin::View& d_cam = pangolin::Display("lidar_lidar_3d")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_w), 1.0,
                   -1.0 * static_cast<double>(view_3d_w) / static_cast<double>(height))
        .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::CreatePanel("cp").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel_w));

    pangolin::Var<bool> solidPairColors("cp.Solid Ref/Tgt colors", true, true);
    pangolin::Var<bool> displayMode("cp.Intensity Color", true, true);
    pangolin::Var<int> tgtPointBoost("cp.Tgt point +", 2, 0, 8);
    pangolin::Var<double> degreeStep("cp.deg step", step.rot_step_deg, 0.01, 4.0);
    pangolin::Var<double> tStepCm("cp.t step(cm)", step.trans_step_m * 100.0, 0.1, 40.0);
    pangolin::Var<int> pointSize("cp.Point Size", kManualPangolinPointSizeDefault, 0,
                                 kManualPangolinPointSizeMax);

    const Eigen::Vector3d e0 = adjuster.current().euler_deg();
    const Eigen::Vector3d t0 = adjuster.current().translation();
    pangolin::Var<double> rollDeg("cp.roll_deg", e0.x(), -360.0, 360.0);
    pangolin::Var<double> pitchDeg("cp.pitch_deg", e0.y(), -360.0, 360.0);
    pangolin::Var<double> yawDeg("cp.yaw_deg", e0.z(), -360.0, 360.0);
    pangolin::Var<double> txM("cp.tx_m", t0.x(), -80.0, 80.0);
    pangolin::Var<double> tyM("cp.ty_m", t0.y(), -80.0, 80.0);
    pangolin::Var<double> tzM("cp.tz_m", t0.z(), -80.0, 80.0);
    pangolin::Var<bool> applyPoseBtn("cp.Apply RPY+T", false, false);

    pangolin::Var<bool> addXdeg("cp.+ x deg", false, false);
    pangolin::Var<bool> minusXdeg("cp.- x deg", false, false);
    pangolin::Var<bool> addYdeg("cp.+ y deg", false, false);
    pangolin::Var<bool> minusYdeg("cp.- y deg", false, false);
    pangolin::Var<bool> addZdeg("cp.+ z deg", false, false);
    pangolin::Var<bool> minusZdeg("cp.- z deg", false, false);
    pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
    pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
    pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
    pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
    pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
    pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

    pangolin::Var<bool> resetBtn("cp.Reset", false, false);
    pangolin::Var<bool> acceptBtn("cp.Accept", false, false);

    // 修复点 2: 移除 kbhit/getchar，改用 Pangolin 回调
    bool extrinsic_changed = false;
    pangolin::RegisterKeyPressCallback('q', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=q ROLL_PLUS"); adjuster.apply(AdjustCmd::ROLL_PLUS, false);   extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('a', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=a ROLL_MINUS"); adjuster.apply(AdjustCmd::ROLL_MINUS, false);  extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('w', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=w PITCH_PLUS"); adjuster.apply(AdjustCmd::PITCH_PLUS, false);  extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('s', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=s PITCH_MINUS"); adjuster.apply(AdjustCmd::PITCH_MINUS, false); extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('e', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=e YAW_PLUS"); adjuster.apply(AdjustCmd::YAW_PLUS, false);    extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('d', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=d YAW_MINUS"); adjuster.apply(AdjustCmd::YAW_MINUS, false);   extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('r', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=r TX_PLUS"); adjuster.apply(AdjustCmd::TX_PLUS, false);     extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('f', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=f TX_MINUS"); adjuster.apply(AdjustCmd::TX_MINUS, false);    extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('t', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=t TY_PLUS"); adjuster.apply(AdjustCmd::TY_PLUS, false);     extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('g', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=g TY_MINUS"); adjuster.apply(AdjustCmd::TY_MINUS, false);    extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('y', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=y TZ_PLUS"); adjuster.apply(AdjustCmd::TZ_PLUS, false);     extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('h', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=h TZ_MINUS"); adjuster.apply(AdjustCmd::TZ_MINUS, false);    extrinsic_changed = true; });
    pangolin::RegisterKeyPressCallback('u', [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=u UNDO"); adjuster.undo();                              extrinsic_changed = true; });
    
    std::optional<ExtrinsicSE3> result;
    bool accepted = false;

    pangolin::RegisterKeyPressCallback(13, [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=Enter ACCEPT"); accepted = true; }); // Enter
    pangolin::RegisterKeyPressCallback(27, [&]() { UNICALIB_INFO("[ManualLidarLidar] Key=Esc QuitAll"); pangolin::QuitAll(); }); // Esc

    size_t ref_n = ref_scan.cloud->size();
    size_t target_n = target_scan.cloud->size();
    std::vector<float> ref_verts, target_verts;
    std::vector<unsigned char> ref_colors, target_colors;
    uploadRefCloud(ref_scan, displayMode.Get(), solidPairColors.Get(), &ref_verts, &ref_colors);
    uploadTargetCloud(target_scan, adjuster.current().SE3_TargetInRef().matrix(), displayMode.Get(),
                      solidPairColors.Get(), &target_verts, &target_colors);

    pangolin::GlBuffer ref_vertexBuffer(pangolin::GlArrayBuffer, ref_verts.size() / 3, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
    pangolin::GlBuffer ref_colorBuffer(pangolin::GlArrayBuffer, ref_colors.size() / 3, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
    pangolin::GlBuffer target_vertexBuffer(pangolin::GlArrayBuffer, target_verts.size() / 3, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
    pangolin::GlBuffer target_colorBuffer(pangolin::GlArrayBuffer, target_colors.size() / 3, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
    ref_vertexBuffer.Upload(ref_verts.data(), sizeof(float) * ref_verts.size(), 0);
    ref_colorBuffer.Upload(ref_colors.data(), sizeof(unsigned char) * ref_colors.size(), 0);
    target_vertexBuffer.Upload(target_verts.data(), sizeof(float) * target_verts.size(), 0);
    target_colorBuffer.Upload(target_colors.data(), sizeof(unsigned char) * target_colors.size(), 0);

    int point_size_val = kManualPangolinPointSizeDefault;
    UNICALIB_INFO("[ManualLidarLidar] 6-DOF: 左栏可填 roll/pitch/yaw(deg) 与 tx/ty/tz(m) 后点 Apply RPY+T 作为起点，再用 +/- 或"
                  " 键盘 q/a w/s e/d、r/f t/g y/h 微调；u 撤销；Enter 接受；Esc 退出。"
                  " 默认 Solid Ref/Tgt：Ref 青绿、Target 橙，易区分（可关用以强度着色）。");

    int loop_count = 0;
    while (!pangolin::ShouldQuit() && !accepted) {
        ++loop_count;
        if (loop_count == 1 || loop_count % 200 == 0) {
            UNICALIB_INFO("[ManualLidarLidar] 渲染循环: loop_count={} should_quit={} accepted={}",
                          loop_count, pangolin::ShouldQuit(), accepted);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 与左侧面板步长一致（否则仅 RegisterKeyPressCallback 仍用会话初值步长）
        {
            ManualAdjustStep st;
            st.rot_step_deg = degreeStep.Get();
            st.trans_step_m = tStepCm.Get() * 0.01;
            st.rot_fast_deg = std::max(0.5, st.rot_step_deg * 10.0);
            st.trans_fast_m = std::max(0.01, st.trans_step_m * 10.0);
            adjuster.set_adjust_step(st);
        }

        if (displayMode.GuiChanged() || solidPairColors.GuiChanged()) {
            uploadRefCloud(ref_scan, displayMode.Get(), solidPairColors.Get(), &ref_verts, &ref_colors);
            uploadTargetCloud(target_scan, adjuster.current().SE3_TargetInRef().matrix(), displayMode.Get(),
                              solidPairColors.Get(), &target_verts, &target_colors);
            ref_vertexBuffer.Upload(ref_verts.data(), sizeof(float) * ref_verts.size(), 0);
            ref_colorBuffer.Upload(ref_colors.data(), sizeof(unsigned char) * ref_colors.size(), 0);
            target_vertexBuffer.Upload(target_verts.data(), sizeof(float) * target_verts.size(), 0);
            target_colorBuffer.Upload(target_colors.data(), sizeof(unsigned char) * target_colors.size(), 0);
        }
        if (pointSize.GuiChanged()) {
            point_size_val = pointSize.Get();
        }

        auto apply_rot = [&](AdjustCmd plus_cmd, AdjustCmd minus_cmd, bool is_plus) {
            adjuster.apply(is_plus ? plus_cmd : minus_cmd, false);
            extrinsic_changed = true;
        };

        if (pangolin::Pushed(addXdeg))    { apply_rot(AdjustCmd::ROLL_PLUS,   AdjustCmd::ROLL_MINUS,   true);  }
        if (pangolin::Pushed(minusXdeg))   { apply_rot(AdjustCmd::ROLL_PLUS,   AdjustCmd::ROLL_MINUS,   false); }
        if (pangolin::Pushed(addYdeg))    { apply_rot(AdjustCmd::PITCH_PLUS,  AdjustCmd::PITCH_MINUS,  true);  }
        if (pangolin::Pushed(minusYdeg))   { apply_rot(AdjustCmd::PITCH_PLUS,  AdjustCmd::PITCH_MINUS,  false); }
        if (pangolin::Pushed(addZdeg))     { apply_rot(AdjustCmd::YAW_PLUS,    AdjustCmd::YAW_MINUS,    true);  }
        if (pangolin::Pushed(minusZdeg))   { apply_rot(AdjustCmd::YAW_PLUS,    AdjustCmd::YAW_MINUS,    false); }
        if (pangolin::Pushed(addXtrans))   { adjuster.apply(AdjustCmd::TX_PLUS,  false); extrinsic_changed = true; }
        if (pangolin::Pushed(minusXtrans)) { adjuster.apply(AdjustCmd::TX_MINUS, false); extrinsic_changed = true; }
        if (pangolin::Pushed(addYtrans))   { adjuster.apply(AdjustCmd::TY_PLUS,  false); extrinsic_changed = true; }
        if (pangolin::Pushed(minusYtrans)) { adjuster.apply(AdjustCmd::TY_MINUS, false); extrinsic_changed = true; }
        if (pangolin::Pushed(addZtrans))   { adjuster.apply(AdjustCmd::TZ_PLUS,  false); extrinsic_changed = true; }
        if (pangolin::Pushed(minusZtrans)) { adjuster.apply(AdjustCmd::TZ_MINUS, false); extrinsic_changed = true; }

        if (pangolin::Pushed(applyPoseBtn)) {
            adjuster.set_pose_rpy_deg_xyz_m(rollDeg.Get(), pitchDeg.Get(), yawDeg.Get(), txM.Get(), tyM.Get(),
                                            tzM.Get());
            extrinsic_changed = true;
        }

        if (pangolin::Pushed(resetBtn) && initial_extrinsic) {
            adjuster.set_initial_extrinsic(*initial_extrinsic);
            extrinsic_changed = true;
        }
        if (pangolin::Pushed(acceptBtn)) {
            UNICALIB_INFO("[ManualLidarLidar] GUI Accept 按钮触发");
            accepted = true;
            break;
        }

        if (extrinsic_changed) {
            uploadTargetCloud(target_scan, adjuster.current().SE3_TargetInRef().matrix(), displayMode.Get(),
                              solidPairColors.Get(), &target_verts, &target_colors);
            target_vertexBuffer.Upload(target_verts.data(), sizeof(float) * target_verts.size(), 0);
            target_colorBuffer.Upload(target_colors.data(), sizeof(unsigned char) * target_colors.size(), 0);
            const Eigen::Vector3d e = adjuster.current().euler_deg();
            const Eigen::Vector3d t = adjuster.current().translation();
            rollDeg = e.x();
            pitchDeg = e.y();
            yawDeg = e.z();
            txM = t.x();
            tyM = t.y();
            tzM = t.z();
            extrinsic_changed = false; // 重置标志
        }

        d_cam.Activate(s_cam);
        glDisable(GL_LIGHTING);
        glPointSize(static_cast<GLfloat>(point_size_val));

        // Ref 点云（Solid 模式下青绿；否则灰/强度）
        ref_colorBuffer.Bind();
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
        ref_vertexBuffer.Bind();
        glVertexPointer(3, GL_FLOAT, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(ref_n));
        glDisableClientState(GL_VERTEX_ARRAY);
        ref_vertexBuffer.Unbind();
        glDisableClientState(GL_COLOR_ARRAY);
        ref_colorBuffer.Unbind();

        glPointSize(static_cast<GLfloat>(point_size_val + std::max(0, tgtPointBoost.Get())));

        // Target 点云（Solid 模式下橙红；略大点便于与 Ref 分层）
        target_colorBuffer.Bind();
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
        target_vertexBuffer.Bind();
        glVertexPointer(3, GL_FLOAT, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(target_n));
        glDisableClientState(GL_VERTEX_ARRAY);
        target_vertexBuffer.Unbind();
        glDisableClientState(GL_COLOR_ARRAY);
        target_colorBuffer.Unbind();

        // 与 LiDAR-Camera Pangolin 一致：支持在运行本程序的终端内按键（不依赖 3D 窗口焦点）
        if (kbhit()) {
            int c = getchar();
            char k = static_cast<char>(c & 0xFF);
            if (k == 'q') {
                adjuster.apply(AdjustCmd::ROLL_PLUS, false);
                extrinsic_changed = true;
            }
            if (k == 'a') {
                adjuster.apply(AdjustCmd::ROLL_MINUS, false);
                extrinsic_changed = true;
            }
            if (k == 'w') {
                adjuster.apply(AdjustCmd::PITCH_PLUS, false);
                extrinsic_changed = true;
            }
            if (k == 's') {
                adjuster.apply(AdjustCmd::PITCH_MINUS, false);
                extrinsic_changed = true;
            }
            if (k == 'e') {
                adjuster.apply(AdjustCmd::YAW_PLUS, false);
                extrinsic_changed = true;
            }
            if (k == 'd') {
                adjuster.apply(AdjustCmd::YAW_MINUS, false);
                extrinsic_changed = true;
            }
            if (k == 'r') {
                adjuster.apply(AdjustCmd::TX_PLUS, false);
                extrinsic_changed = true;
            }
            if (k == 'f') {
                adjuster.apply(AdjustCmd::TX_MINUS, false);
                extrinsic_changed = true;
            }
            if (k == 't') {
                adjuster.apply(AdjustCmd::TY_PLUS, false);
                extrinsic_changed = true;
            }
            if (k == 'g') {
                adjuster.apply(AdjustCmd::TY_MINUS, false);
                extrinsic_changed = true;
            }
            if (k == 'y') {
                adjuster.apply(AdjustCmd::TZ_PLUS, false);
                extrinsic_changed = true;
            }
            if (k == 'h') {
                adjuster.apply(AdjustCmd::TZ_MINUS, false);
                extrinsic_changed = true;
            }
            if (k == 'u') {
                adjuster.undo();
                extrinsic_changed = true;
            }
            if (c == 13) {
                accepted = true;
                break;
            }
            if (c == 27) break;
        }

        // 在 3D 视口叠加说明（与 LiDAR-Cam 在图像上画字一致，避免用户误以为只能点面板）
        {
            pangolin::Viewport& v = d_cam.v;
            auto& font = pangolin::default_font();
            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            float y = v.t() - 8 - font.Height();
            float x = v.l + 8;
            const ExtrinsicSE3& cur = adjuster.current();
            Eigen::Vector3d t = cur.POS_TargetInRef;
            Eigen::Vector3d rpy = cur.euler_deg();
            font.Text("T(m): %.3f %.3f %.3f  RPY(deg): %.2f %.2f %.2f", t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z())
                .DrawWindow(x, y);
            y -= font.Height() * 1.25f;
            font.Text("Solid: Ref=cyan-green  Target=orange  |  Panel: Apply RPY+T then +/-")
                .DrawWindow(x, y);
            y -= font.Height() * 1.25f;
            font.Text("Keys: Q/A Roll  W/S Pitch  E/D Yaw  |  R/F Tx  T/G Ty  Y/H Tz  |  U Undo  Enter  Esc")
                .DrawWindow(x, y);
            y -= font.Height() * 1.25f;
            font.Text("Or type keys in this terminal (same as LiDAR-Cam Pangolin).").DrawWindow(x, y);
            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
    UNICALIB_INFO("[ManualLidarLidar] 跳出循环: loop_count={} should_quit={} accepted={}",
                  loop_count, pangolin::ShouldQuit(), accepted);

    if (accepted) {
        result = adjuster.current();
        UNICALIB_INFO("[ManualLidarLidar] 用户接受并退出");
    } else {
        UNICALIB_INFO("[ManualLidarLidar] 用户取消");
    }

    UNICALIB_INFO("[ManualLidarLidar] DestroyWindow 开始");
    pangolin::DestroyWindow("LiDAR-LiDAR Manual Calibration");
    UNICALIB_INFO("[ManualLidarLidar] DestroyWindow 完成");
    return result;
}

}  // namespace ns_unicalib

#endif  // UNICALIB_WITH_PANGOLIN
