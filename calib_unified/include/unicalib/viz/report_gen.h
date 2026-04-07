#pragma once
/**
 * UniCalib Unified — HTML 标定报告生成器
 * 生成包含以下内容的交互式 HTML 报告:
 *   - 标定参数汇总表
 *   - Allan 方差图 (base64 内嵌 PNG)
 *   - 重投影误差分布图
 *   - 外参可视化 (变换矩阵 + 欧拉角 + 平移)
 *   - 时间偏移可视化
 *   - 验证指标
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <map>
#include <optional>

// Forward declaration to break circular dependency
namespace ns_unicalib {
struct EdgeAlignmentScore;
}

namespace ns_unicalib {

// ===================================================================
// 报告内容数据结构
// ===================================================================
struct IMUIntrinsicReport {
    std::string sensor_id;
    IMUIntrinsics intrinsics;
    AllanResult   allan_result;
    std::optional<SixPositionResult> six_pos_result;
    std::string allan_plot_path;    // Allan 偏差图 PNG 路径
};

struct CameraIntrinsicReport {
    std::string sensor_id;
    CameraIntrinsics intrinsics;
    std::string reproj_error_plot_path;
    std::vector<double> per_image_errors;
};

struct ExtrinsicReport {
    std::string ref_id, target_id;
    CalibType calib_type;
    ExtrinsicSE3 result;
    std::string projection_image_path;  // 验证图像路径 (可选)
    std::optional<double> edge_alignment_score;
    // EdgeAlignmentScore 定义在 lidar_camera_calib.h 中
};

// ===================================================================
// HTML 报告生成器
// ===================================================================
class ReportGenerator {
public:
    struct Config {
        std::string title = "UniCalib 标定报告";
        bool embed_images = true;       // 将图像 base64 内嵌到 HTML
        bool dark_theme   = false;
        std::string css_override;       // 自定义 CSS
        bool verbose = false;
    };

    explicit ReportGenerator(const Config& cfg) : cfg_(cfg) {}
    ReportGenerator() : cfg_(Config{}) {}

    // 设置内参报告数据
    void add_imu_intrinsic(const IMUIntrinsicReport& r) {
        imu_reports_.push_back(r);
    }
    void add_camera_intrinsic(const CameraIntrinsicReport& r) {
        cam_reports_.push_back(r);
    }
    void add_extrinsic(const ExtrinsicReport& r) {
        extrin_reports_.push_back(r);
    }

    // 设置全局信息
    void set_system_config(const SystemConfig& cfg) { sys_cfg_ = cfg; }
    void set_timestamp(const std::string& ts) { timestamp_ = ts; }

    // 生成 HTML 报告
    bool generate(const std::string& output_html_path);

    // 生成简短的终端摘要
    void print_terminal_summary(const CalibParamManager& params) const;

    // 生成 YAML 摘要 (机器可读)
    bool generate_yaml(const CalibParamManager& params,
                       const std::string& output_yaml_path);

private:
    Config cfg_;
    std::vector<IMUIntrinsicReport>    imu_reports_;
    std::vector<CameraIntrinsicReport> cam_reports_;
    std::vector<ExtrinsicReport>       extrin_reports_;
    SystemConfig sys_cfg_;
    std::string timestamp_;

    // HTML 生成辅助函数
    std::string gen_header() const;
    std::string gen_imu_section(const IMUIntrinsicReport& r) const;
    std::string gen_camera_section(const CameraIntrinsicReport& r) const;
    std::string gen_extrinsic_section(const ExtrinsicReport& r) const;
    std::string gen_footer() const;

    // 将图像文件转为 base64 嵌入
    static std::string image_to_base64(const std::string& path);
    static std::string build_img_tag(const std::string& path,
                                     const std::string& alt,
                                     bool embed = true);

    // SE3 转文本表示
    static std::string se3_to_html_table(const ExtrinsicSE3& extrin);
    static std::string mat4x4_to_html(const Eigen::Matrix4d& T);
};

}  // namespace ns_unicalib
