/**
 * UniCalib — LiDAR-Camera 手动标定 Pangolin 窗口实现 (方案 B)
 * 仅当 UNICALIB_WITH_PANGOLIN 时编译。
 */

#if UNICALIB_WITH_PANGOLIN

#include "unicalib/viz/manual_lidar_cam_window.h"
#include "unicalib/viz/manual_pangolin_ui_scale.h"
#include "unicalib/viz/projector_lidar.h"
#include "unicalib/common/logger.h"
#include "unicalib/viz/pangolin_compat.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/utils/params.h>
#include <pangolin/var/var.h>
#include <pangolin/gl/gl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>

namespace ns_unicalib {

namespace {

static bool kbhit() {
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

}  // namespace

std::optional<ExtrinsicSE3> run_lidar_cam_pangolin_panel(
    ManualExtrinsicAdjuster& adjuster,
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ManualAdjustStep& step,
    const ExtrinsicSE3* initial_extrinsic) {

    if (!scan.cloud || scan.cloud->empty() || image.empty()) {
        UNICALIB_WARN("[ManualPangolin] 点云或图像为空");
        return std::nullopt;
    }

    const int width = image.cols;
    const int height = image.rows;
    const int panel_w = kManualPangolinPanelWidthPx;

    LidarProjector projector;
    if (!projector.loadPointCloud(*scan.cloud)) {
        UNICALIB_WARN("[ManualPangolin] loadPointCloud 失败");
        return std::nullopt;
    }

    pangolin::Params win_params;
    win_params.Set("default_font_size", kManualPangolinDefaultFontPx);
    pangolin::CreateWindowAndBind("LiDAR-Camera Manual Calibration", width + panel_w, height, win_params);
    glEnable(GL_DEPTH_TEST);

    pangolin::View& project_view = pangolin::Display("project")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_w), 1.0,
                   -1.0 * static_cast<double>(width) / static_cast<double>(height))
        .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::CreatePanel("cp")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel_w));

    pangolin::Var<bool> displayMode("cp.Intensity Color", false, true);
    pangolin::Var<bool> filterMode("cp.Overlap Filter", false, true);
    pangolin::Var<double> degreeStep("cp.deg step", step.rot_step_deg, 0.01, 4.0);
    pangolin::Var<double> tStepCm("cp.t step(cm)", step.trans_step_m * 100.0, 0.1, 40.0);
    pangolin::Var<int> pointSize("cp.point size", kManualPangolinPointSizeDefault, 1,
                                 kManualPangolinPointSizeMax);

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
    pangolin::Var<bool> saveBtn("cp.Save Image", false, false);
    pangolin::Var<bool> acceptBtn("cp.Accept", false, false);

    pangolin::GlTexture imageTexture(static_cast<GLint>(width), static_cast<GLint>(height),
                                      GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    std::optional<ExtrinsicSE3> result;
    bool accepted = false;

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (displayMode) {
            projector.setDisplayMode(true);
        } else {
            projector.setDisplayMode(false);
        }
        if (filterMode) {
            projector.setFilterMode(true);
        } else {
            projector.setFilterMode(false);
        }

        if (pointSize.GuiChanged()) {
            projector.setPointSize(pointSize.Get());
        }

        auto apply_rot = [&](AdjustCmd plus_cmd, AdjustCmd minus_cmd, bool is_plus) {
            adjuster.apply(is_plus ? plus_cmd : minus_cmd, false);
        };
        if (pangolin::Pushed(addXdeg))    apply_rot(AdjustCmd::ROLL_PLUS,   AdjustCmd::ROLL_MINUS,   true);
        if (pangolin::Pushed(minusXdeg))  apply_rot(AdjustCmd::ROLL_PLUS,   AdjustCmd::ROLL_MINUS,   false);
        if (pangolin::Pushed(addYdeg))    apply_rot(AdjustCmd::PITCH_PLUS,  AdjustCmd::PITCH_MINUS,  true);
        if (pangolin::Pushed(minusYdeg))  apply_rot(AdjustCmd::PITCH_PLUS,  AdjustCmd::PITCH_MINUS,  false);
        if (pangolin::Pushed(addZdeg))    apply_rot(AdjustCmd::YAW_PLUS,    AdjustCmd::YAW_MINUS,    true);
        if (pangolin::Pushed(minusZdeg))  apply_rot(AdjustCmd::YAW_PLUS,    AdjustCmd::YAW_MINUS,    false);
        if (pangolin::Pushed(addXtrans))  adjuster.apply(AdjustCmd::TX_PLUS,  false);
        if (pangolin::Pushed(minusXtrans)) adjuster.apply(AdjustCmd::TX_MINUS, false);
        if (pangolin::Pushed(addYtrans))  adjuster.apply(AdjustCmd::TY_PLUS,  false);
        if (pangolin::Pushed(minusYtrans)) adjuster.apply(AdjustCmd::TY_MINUS, false);
        if (pangolin::Pushed(addZtrans))  adjuster.apply(AdjustCmd::TZ_PLUS,  false);
        if (pangolin::Pushed(minusZtrans)) adjuster.apply(AdjustCmd::TZ_MINUS, false);

        if (pangolin::Pushed(resetBtn) && initial_extrinsic) {
            adjuster.set_initial_extrinsic(*initial_extrinsic);
        }
        if (pangolin::Pushed(acceptBtn)) {
            accepted = true;
            break;
        }

        static int save_frame_num = 0;
        if (pangolin::Pushed(saveBtn)) {
            const ExtrinsicSE3& cur_save = adjuster.current();
            Eigen::Matrix4d T_ltc = cur_save.SE3_TargetInRef().matrix();
            cv::Mat save_img = projector.projectToImage(image, cam_intrin, T_ltc);
            std::string path = "manual_calib_save_" + std::to_string(save_frame_num++) + ".png";
            if (cv::imwrite(path, save_img))
                UNICALIB_INFO("[ManualPangolin] 已保存: {}", path);
        }

        if (kbhit()) {
            int c = getchar();
            char k = static_cast<char>(c & 0xFF);
            if (k == 'q') adjuster.apply(AdjustCmd::ROLL_PLUS,   false);
            if (k == 'a') adjuster.apply(AdjustCmd::ROLL_MINUS,  false);
            if (k == 'w') adjuster.apply(AdjustCmd::PITCH_PLUS, false);
            if (k == 's') adjuster.apply(AdjustCmd::PITCH_MINUS, false);
            if (k == 'e') adjuster.apply(AdjustCmd::YAW_PLUS,    false);
            if (k == 'd') adjuster.apply(AdjustCmd::YAW_MINUS,  false);
            if (k == 'r') adjuster.apply(AdjustCmd::TX_PLUS,    false);
            if (k == 'f') adjuster.apply(AdjustCmd::TX_MINUS,   false);
            if (k == 't') adjuster.apply(AdjustCmd::TY_PLUS,    false);
            if (k == 'g') adjuster.apply(AdjustCmd::TY_MINUS,   false);
            if (k == 'y') adjuster.apply(AdjustCmd::TZ_PLUS,    false);
            if (k == 'h') adjuster.apply(AdjustCmd::TZ_MINUS,   false);
            if (k == 'u') adjuster.undo();
            if (c == 13) { accepted = true; break; }
            if (c == 27) break;
        }

        const ExtrinsicSE3& cur = adjuster.current();
        // Pipeline 存的是 T_lidar_to_cam (p_cam = T*p_lidar)，Projector 期望的也是 T_lidar_to_cam，直接取 matrix
        Eigen::Matrix4d T_lidar_to_cam = cur.SE3_TargetInRef().matrix();

        cv::Mat frame = projector.projectToImage(image, cam_intrin, T_lidar_to_cam);
        imageTexture.Upload(frame.data, GL_BGR, GL_UNSIGNED_BYTE);

        project_view.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);
        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

    if (accepted) {
        result = adjuster.current();
        UNICALIB_INFO("[ManualPangolin] 用户接受并退出");
    } else {
        UNICALIB_INFO("[ManualPangolin] 用户取消");
    }

    pangolin::DestroyWindow("LiDAR-Camera Manual Calibration");
    return result;
}

}  // namespace ns_unicalib

#endif  // UNICALIB_WITH_PANGOLIN
