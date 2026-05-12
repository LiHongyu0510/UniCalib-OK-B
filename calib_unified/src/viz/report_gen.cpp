/**
 * UniCalib Unified — HTML 报告生成器实现
 */

#include "unicalib/viz/report_gen.h"
#include "unicalib/common/logger.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <cmath>
#include <algorithm>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===================================================================
// 内部工具
// ===================================================================
namespace {

std::string mat_to_base64(const cv::Mat& img) {
    std::vector<uchar> buf;
    cv::imencode(".png", img, buf);
    static const char* B64 =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string r;
    r.reserve(((buf.size() + 2) / 3) * 4 + 4);
    for (size_t i = 0; i < buf.size(); i += 3) {
        uint32_t v = (buf[i] << 16);
        v |= (i+1 < buf.size() ? buf[i+1] << 8 : 0);
        v |= (i+2 < buf.size() ? buf[i+2]      : 0);
        r += B64[(v>>18)&63]; r += B64[(v>>12)&63];
        r += (i+1<buf.size() ? B64[(v>>6)&63] : '=');
        r += (i+2<buf.size() ? B64[v&63]      : '=');
    }
    return r;
}

std::string current_time_str() {
    auto t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return buf;
}

std::string fmt_sci(double v, int p = 4) {
    std::ostringstream ss;
    ss << std::scientific << std::setprecision(p) << v;
    return ss.str();
}

std::string fmt_fix(double v, int p = 4) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(p) << v;
    return ss.str();
}

cv::Mat draw_allan_plot_impl(const AllanResult& allan) {
    const int W = 680, H = 420;
    const int ML = 75, MB = 50, MR = 30, MT = 40;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(248,248,248));

    double tmin=1e30, tmax=-1e30, amin=1e30, amax=-1e30;
    for (int ax = 0; ax < 3; ++ax) {
        for (size_t i = 0; i < allan.gyro[ax].taus.size(); ++i) {
            if (allan.gyro[ax].taus[i] > 0 && allan.gyro[ax].adevs[i] > 0) {
                tmin = std::min(tmin, allan.gyro[ax].taus[i]);
                tmax = std::max(tmax, allan.gyro[ax].taus[i]);
                amin = std::min(amin, allan.gyro[ax].adevs[i]);
                amax = std::max(amax, allan.gyro[ax].adevs[i]);
            }
        }
    }
    if (tmin >= tmax || amin >= amax) return img;

    int pw = W-ML-MR, ph = H-MB-MT;
    auto toPx = [&](double tau, double ad) -> cv::Point {
        double x = (std::log10(tau)-std::log10(tmin))/(std::log10(tmax)-std::log10(tmin)+1e-9);
        double y = (std::log10(ad) -std::log10(amin))/(std::log10(amax)-std::log10(amin)+1e-9);
        return {ML+static_cast<int>(x*pw), MT+static_cast<int>((1-y)*ph)};
    };

    cv::rectangle(img, {ML,MT}, {W-MR,H-MB}, cv::Scalar(200,200,200), 1);
    const cv::Scalar clr[3] = {{200,60,60},{60,170,60},{60,60,200}};
    const char* ax_nm[3] = {"X","Y","Z"};
    for (int ax = 0; ax < 3; ++ax) {
        const auto& r = allan.gyro[ax];
        for (size_t i = 1; i < r.taus.size(); ++i)
            if (r.adevs[i]>0 && r.adevs[i-1]>0)
                cv::line(img, toPx(r.taus[i-1],r.adevs[i-1]), toPx(r.taus[i],r.adevs[i]), clr[ax], 2);
        cv::line(img, {W-MR-90,MT+10+ax*16}, {W-MR-72,MT+10+ax*16}, clr[ax], 2);
        cv::putText(img, std::string("Gyro-")+ax_nm[ax], {W-MR-68,MT+14+ax*16},
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, clr[ax], 1);
    }
    cv::putText(img, "Allan Deviation (Gyroscope)", {ML,MT-8},
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(30,30,30), 1);
    cv::putText(img, "tau [s]", {ML+pw/2-20,H-8},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(80,80,80), 1);
    return img;
}

std::string cv_img_to_html_tag(const cv::Mat& img, int w=640) {
    if (img.empty()) return "<p><em>[图像不可用]</em></p>";
    return "<img src='data:image/png;base64," + mat_to_base64(img) +
           "' width='" + std::to_string(w) +
           "' style='max-width:100%;border:1px solid #ddd;border-radius:4px'>";
}

// 通用 CSS 头部
std::string html_head(const std::string& title, const std::string& accent_color = "#4a90d9") {
    std::ostringstream ss;
    ss << "<!DOCTYPE html><html lang='zh-CN'><head>\n"
       << "<meta charset='UTF-8'><title>" << title << "</title>\n"
       << "<style>\n"
       << "body{font-family:'Segoe UI',Helvetica,sans-serif;max-width:1000px;"
       << "margin:36px auto;background:#f7f8fa;color:#1a1a2e;padding:0 20px}\n"
       << "h1{border-bottom:3px solid " << accent_color << ";padding-bottom:10px}\n"
       << "h2{color:" << accent_color << ";margin-top:26px}\n"
       << ".card{background:#fff;border-radius:8px;padding:18px 22px;"
       << "box-shadow:0 2px 8px rgba(0,0,0,0.07);margin:16px 0}\n"
       << "table{border-collapse:collapse;width:100%}\n"
       << "th,td{padding:8px 12px;text-align:left;border-bottom:1px solid #eee}\n"
       << "th{background:" << accent_color << ";color:#fff}\n"
       << "tr:hover{background:#f0f8ff}\n"
       << ".ok{color:#27ae60;font-weight:bold} .warn{color:#e67e22;font-weight:bold}\n"
       << ".err{color:#e74c3c;font-weight:bold} .note{color:#888;font-size:.9em}\n"
       << "pre{background:#f4f4f8;padding:12px;border-radius:4px;font-size:.88em;"
       << "overflow-x:auto;white-space:pre-wrap}\n"
       << "</style></head><body>\n";
    return ss.str();
}

}  // anonymous namespace

// ===================================================================
// generate() — 主报告生成函数
// ===================================================================
bool ReportGenerator::generate(const std::string& output_html_path) {
    UNICALIB_INFO("生成 HTML 报告: {}", output_html_path);
    fs::create_directories(fs::path(output_html_path).parent_path());

    std::ostringstream html;
    html << gen_header();

    for (const auto& r : imu_reports_)
        html << gen_imu_section(r);

    for (const auto& r : cam_reports_)
        html << gen_camera_section(r);

    for (const auto& r : extrin_reports_)
        html << gen_extrinsic_section(r);

    html << gen_footer();

    std::ofstream ofs(output_html_path);
    if (!ofs.is_open()) {
        UNICALIB_ERROR("无法写入报告文件: {}", output_html_path);
        return false;
    }
    ofs << html.str();
    UNICALIB_INFO("报告已保存: {}", output_html_path);
    return true;
}

// ===================================================================
std::string ReportGenerator::gen_header() const {
    std::ostringstream ss;
    ss << html_head(cfg_.title, "#3498db");
    ss << "<h1>" << cfg_.title << "</h1>\n";
    ss << "<p class='note'>生成时间: " << current_time_str() << "</p>\n";

    // 传感器概览
    if (!sys_cfg_.sensors.empty()) {
        ss << "<div class='card'><h2>传感器配置</h2><table>"
           << "<tr><th>ID</th><th>类型</th><th>话题</th></tr>\n";
        for (const auto& s : sys_cfg_.sensors) {
            ss << "<tr><td>" << s.sensor_id << "</td>"
               << "<td>" << sensor_type_to_str(s.type) << "</td>"
               << "<td>" << s.topic << "</td></tr>\n";
        }
        ss << "</table></div>\n";
    }
    return ss.str();
}

std::string ReportGenerator::gen_imu_section(const IMUIntrinsicReport& r) const {
    std::ostringstream ss;
    ss << "<div class='card'>\n<h2>IMU 内参 [" << r.sensor_id << "]</h2>\n";
    ss << "<table><tr><th>参数</th><th>值</th><th>单位</th></tr>\n";

    const auto& I = r.intrinsics;
    ss << "<tr><td>陀螺噪声密度 N_g</td><td>" << fmt_sci(I.noise_gyro)
       << "</td><td>rad/s/√Hz</td></tr>\n";
    ss << "<tr><td>陀螺零偏不稳定性 B_g</td><td>" << fmt_sci(I.bias_instab_gyro)
       << "</td><td>rad/s</td></tr>\n";
    ss << "<tr><td>加速度计噪声密度 N_a</td><td>" << fmt_sci(I.noise_acce)
       << "</td><td>m/s²/√Hz</td></tr>\n";
    ss << "<tr><td>加速度计零偏不稳定性 B_a</td><td>" << fmt_sci(I.bias_instab_acce)
       << "</td><td>m/s²</td></tr>\n";
    ss << "<tr><td>使用样本</td><td>" << I.num_samples_used << "</td><td>帧</td></tr>\n";
    ss << "</table>\n";

    // Allan 图
    if (!r.allan_result.gyro[0].taus.empty()) {
        cv::Mat plot = draw_allan_plot_impl(r.allan_result);
        ss << "<h2>Allan 偏差曲线</h2>\n";
        ss << cv_img_to_html_tag(plot, 660) << "\n";
        ss << "<ul class='note'>\n";
        const char* axes[] = {"X","Y","Z"};
        for (int ax = 0; ax < 3; ++ax) {
            const auto& ar = r.allan_result.gyro[ax];
            ss << "<li><b>Axis-" << axes[ax] << "</b>: ARW="
               << fmt_sci(ar.angle_random_walk) << " rad/s/√Hz, "
               << "BI=" << fmt_sci(ar.bias_instability) << " rad/s</li>\n";
        }
        ss << "</ul>\n";
    }

    // Kalibr 格式参数
    ss << "<h2>推荐配置 (YAML)</h2><pre>";
    ss << "imu:\n";
    ss << "  gyroscope_noise_density: " << fmt_sci(I.noise_gyro) << "\n";
    ss << "  gyroscope_random_walk:   " << fmt_sci(I.bias_instab_gyro) << "\n";
    ss << "  accelerometer_noise_density: " << fmt_sci(I.noise_acce) << "\n";
    ss << "  accelerometer_random_walk:   " << fmt_sci(I.bias_instab_acce) << "\n";
    ss << "  update_rate: 200  # [Hz]\n";
    ss << "</pre></div>\n";
    return ss.str();
}

std::string ReportGenerator::gen_camera_section(const CameraIntrinsicReport& r) const {
    std::ostringstream ss;
    ss << "<div class='card'>\n<h2>相机内参 [" << r.sensor_id << "]</h2>\n";
    const auto& I = r.intrinsics;
    std::string model_str = (I.model == CameraIntrinsics::Model::FISHEYE) ?
                             "鱼眼 KB4" : "针孔 Radtan";
    std::string rms_cls = (I.rms_reproj_error < 1.0) ? "ok" : "warn";

    ss << "<table><tr><th>参数</th><th>值</th></tr>\n";
    ss << "<tr><td>相机模型</td><td>" << model_str << "</td></tr>\n";
    ss << "<tr><td>分辨率</td><td>" << I.width << "×" << I.height << " px</td></tr>\n";
    ss << "<tr><td>fx</td><td>" << fmt_fix(I.fx) << " px</td></tr>\n";
    ss << "<tr><td>fy</td><td>" << fmt_fix(I.fy) << " px</td></tr>\n";
    ss << "<tr><td>cx</td><td>" << fmt_fix(I.cx) << " px</td></tr>\n";
    ss << "<tr><td>cy</td><td>" << fmt_fix(I.cy) << " px</td></tr>\n";
    ss << "<tr><td>RMS 重投影误差</td><td class='" << rms_cls << "'>"
       << fmt_fix(I.rms_reproj_error) << " px</td></tr>\n";
    ss << "<tr><td>使用图像</td><td>" << I.num_images_used << "</td></tr>\n";

    if (!I.dist_coeffs.empty()) {
        ss << "<tr><td>畸变系数</td><td>[";
        for (size_t i = 0; i < I.dist_coeffs.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << fmt_fix(I.dist_coeffs[i], 6);
        }
        ss << "]</td></tr>\n";
    }
    ss << "</table>\n";

    // 逐帧误差柱状图
    if (!r.per_image_errors.empty()) {
        const int W = 660, H = 180;
        cv::Mat bar(H, W, CV_8UC3, cv::Scalar(248,248,248));
        double max_e = *std::max_element(r.per_image_errors.begin(), r.per_image_errors.end());
        max_e = std::max(max_e, 0.01);
        int n = static_cast<int>(r.per_image_errors.size());
        int bw = std::max(1, W / std::max(1, n));
        for (int i = 0; i < n; ++i) {
            int bh = static_cast<int>(r.per_image_errors[i] / max_e * (H-30));
            cv::Scalar c = (r.per_image_errors[i] < 1.0) ?
                cv::Scalar(60,160,60) : cv::Scalar(60,60,200);
            cv::rectangle(bar, {i*bw, H-30-bh}, {i*bw+bw-1, H-30}, c, cv::FILLED);
        }
        cv::putText(bar, "Per-image Reprojection Error (px)",
                    {4,16}, cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(40,40,40), 1);
        ss << cv_img_to_html_tag(bar, 660) << "\n";
    }

    ss << "</div>\n";
    return ss.str();
}

std::string ReportGenerator::gen_extrinsic_section(const ExtrinsicReport& r) const {
    std::ostringstream ss;
    ss << "<div class='card'>\n<h2>外参 [" << r.ref_id << " → " << r.target_id << "]</h2>\n";
    const auto& E = r.result;
    auto t = E.POS_TargetInRef;
    auto euler = E.euler_deg();
    std::string conv_cls = E.is_converged ? "ok" : "err";

    ss << "<table><tr><th>参数</th><th>值</th></tr>\n";
    ss << "<tr><td>参考传感器</td><td>" << E.ref_sensor_id << "</td></tr>\n";
    ss << "<tr><td>目标传感器</td><td>" << E.target_sensor_id << "</td></tr>\n";
    ss << "<tr><td>平移 (x,y,z) [m]</td><td>["
       << fmt_fix(t.x(),6) << ", " << fmt_fix(t.y(),6) << ", " << fmt_fix(t.z(),6)
       << "]</td></tr>\n";
    ss << "<tr><td>旋转 RPY [°]</td><td>["
       << fmt_fix(euler[0],3) << "°, " << fmt_fix(euler[1],3) << "°, "
       << fmt_fix(euler[2],3) << "°]</td></tr>\n";
    ss << "<tr><td>时间偏移</td><td>" << fmt_fix(E.time_offset_s, 6) << " s</td></tr>\n";
    ss << "<tr><td>RMS 残差</td><td>" << fmt_fix(E.residual_rms, 5) << "</td></tr>\n";
    ss << "<tr><td>收敛状态</td><td class='" << conv_cls << "'>"
       << (E.is_converged ? "✓ 已收敛" : "✗ 未收敛") << "</td></tr>\n";
    ss << "</table>\n";
    ss << se3_to_html_table(E);
    ss << "</div>\n";
    return ss.str();
}

std::string ReportGenerator::gen_footer() const {
    std::ostringstream ss;
    ss << "<div class='card' style='background:#e8f4fd'>\n";
    ss << "<p class='note'>报告由 UniCalib v2.0 自动生成 — "
       << current_time_str() << "</p>\n";
    ss << "<p class='note'>源码: UniCalib/calib_unified</p>\n";
    ss << "</div>\n</body></html>\n";
    return ss.str();
}

// ===================================================================
// se3_to_html_table
// ===================================================================
std::string ReportGenerator::se3_to_html_table(const ExtrinsicSE3& extrin) {
    Sophus::SE3d se3 = extrin.SE3_TargetInRef();
    return mat4x4_to_html(se3.matrix());
}

std::string ReportGenerator::mat4x4_to_html(const Eigen::Matrix4d& T) {
    std::ostringstream ss;
    ss << "<h3>变换矩阵 T (4×4)</h3>\n<pre>";
    for (int r = 0; r < 4; ++r) {
        ss << "[ ";
        for (int c = 0; c < 4; ++c) {
            ss << std::setw(13) << std::fixed << std::setprecision(8) << T(r,c) << " ";
        }
        ss << "]\n";
    }
    ss << "</pre>\n";
    return ss.str();
}

// ===================================================================
// image_to_base64 / build_img_tag
// ===================================================================
std::string ReportGenerator::image_to_base64(const std::string& path) {
    cv::Mat img = cv::imread(path);
    if (img.empty()) return "";
    return mat_to_base64(img);
}

std::string ReportGenerator::build_img_tag(const std::string& path,
                                            const std::string& alt,
                                            bool embed) {
    if (!embed) return "<img src='" + path + "' alt='" + alt + "'>";
    auto b64 = image_to_base64(path);
    if (b64.empty()) return "<p><em>[图像不可用]</em></p>";
    return "<img src='data:image/png;base64," + b64 +
           "' alt='" + alt + "' style='max-width:100%'>";
}

// ===================================================================
// print_terminal_summary
// ===================================================================
void ReportGenerator::print_terminal_summary(const CalibParamManager& params) const {
    params.print_summary();
}

// ===================================================================
// generate_yaml
// ===================================================================
bool ReportGenerator::generate_yaml(const CalibParamManager& params,
                                     const std::string& output_yaml_path) {
    try {
        params.save_yaml(output_yaml_path);
        return true;
    } catch (const std::exception& e) {
        UNICALIB_WARN("生成 YAML 报告失败: {} ({})", output_yaml_path, e.what());
        return false;
    } catch (...) {
        UNICALIB_WARN("生成 YAML 报告失败: {} (未知异常)", output_yaml_path);
        return false;
    }
}

}  // namespace ns_unicalib
