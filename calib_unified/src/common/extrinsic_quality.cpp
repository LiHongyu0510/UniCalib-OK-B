#include "unicalib/common/extrinsic_quality.h"
#include "unicalib/common/logger.h"

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>

namespace ns_unicalib {

static int metric_level(double value, double good_thresh, double acceptable_thresh, bool lower_is_better) {
    if (value < 0 || !std::isfinite(value)) return -1;
    if (lower_is_better) {
        if (value <= good_thresh) return 2;
        if (value <= acceptable_thresh) return 1;
        return 0;
    }
    if (value >= good_thresh) return 2;
    if (value >= acceptable_thresh) return 1;
    return 0;
}

ExtrinsicQualityReport assess_lidar_cam_quality(
    double ncc,
    double rms_px,
    double inlier_ratio,
    double chamfer_px,
    double edge_overlap_ratio,
    const QualityThresholds& thresholds) {

    ExtrinsicQualityReport report;
    report.ncc = ncc;
    report.rms_px = rms_px;
    report.inlier_ratio = inlier_ratio;
    report.chamfer_px = chamfer_px;
    report.edge_overlap_ratio = edge_overlap_ratio;

    const int ncc_level = metric_level(ncc, thresholds.ncc_good, thresholds.ncc_acceptable, false);
    const int rms_level = metric_level(rms_px, thresholds.rms_good_px, thresholds.rms_acceptable_px, true);
    const int inlier_level = metric_level(
        inlier_ratio, thresholds.inlier_ratio_good, thresholds.inlier_ratio_acceptable, false);
    const int chamfer_level = metric_level(
        chamfer_px, thresholds.chamfer_good_px, thresholds.chamfer_acceptable_px, true);

    if (ncc_level == 0)
        report.suggestions.push_back("NCC 偏低，请检查初值、光照或边缘丰富的场景");
    else if (ncc_level == 1)
        report.suggestions.push_back("NCC 处于可接受范围，建议增加多帧或改善边缘场景");

    if (rms_level == 0)
        report.suggestions.push_back("重投影 RMS 偏大，建议检查角点检测或初值");
    else if (rms_level == 1)
        report.suggestions.push_back("重投影 RMS 可接受，可尝试增加标定帧或精化角点");

    if (inlier_level == 0)
        report.suggestions.push_back("内点率较低，建议检查匹配或初值");

    if (chamfer_level == 0)
        report.suggestions.push_back("边缘 Chamfer 距离偏大，投影边缘与图像边缘未对齐");
    else if (chamfer_level == 1)
        report.suggestions.push_back("边缘对齐可接受，可尝试边缘法精化或增加帧数");

    struct WeightedMetric { int level; double weight; };
    std::vector<WeightedMetric> parts;
    if (ncc_level >= 0) parts.push_back({ncc_level, 0.30});
    if (rms_level >= 0) parts.push_back({rms_level, 0.30});
    if (inlier_level >= 0) parts.push_back({inlier_level, 0.15});
    if (chamfer_level >= 0) parts.push_back({chamfer_level, 0.25});

    double confidence = 0.0;
    double weight_sum = 0.0;
    for (const auto& p : parts) {
        confidence += (static_cast<double>(p.level) / 2.0) * p.weight;
        weight_sum += p.weight;
    }
    if (weight_sum > 1e-9)
        report.confidence = confidence / weight_sum;
    else
        report.confidence = 0.0;

    if (parts.empty()) {
        report.verdict = QualityVerdict::UNKNOWN;
        report.reasons.push_back("无可用质量指标");
    } else if (report.confidence >= 0.8) {
        report.verdict = QualityVerdict::GOOD;
    } else if (report.confidence >= 0.5) {
        report.verdict = QualityVerdict::ACCEPTABLE;
    } else {
        report.verdict = QualityVerdict::BAD;
    }

    UNICALIB_INFO(
        "[Quality] verdict={} confidence={:.2f} ncc={:.3f} rms={:.2f}px chamfer={:.2f}px overlap={:.3f}",
        quality_verdict_str(report.verdict), report.confidence,
        report.ncc, report.rms_px, report.chamfer_px, report.edge_overlap_ratio);

    return report;
}

bool save_quality_report_yaml(
    const std::string& path,
    const std::string& calibration_type,
    const std::string& stage,
    const std::string& reference_sensor,
    const std::string& target_sensor,
    const std::string& method_used,
    bool has_valid_extrinsic,
    const ExtrinsicQualityReport& report,
    const QualityThresholds& thresholds) {

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "calibration_type" << YAML::Value << calibration_type;
    out << YAML::Key << "stage" << YAML::Value << stage;
    out << YAML::Key << "reference_sensor" << YAML::Value << reference_sensor;
    out << YAML::Key << "target_sensor" << YAML::Value << target_sensor;
    out << YAML::Key << "method_used" << YAML::Value << method_used;
    out << YAML::Key << "has_valid_extrinsic" << YAML::Value << has_valid_extrinsic;

    out << YAML::Key << "metrics" << YAML::BeginMap;
    out << YAML::Key << "ncc" << YAML::Value << report.ncc;
    out << YAML::Key << "rms_px" << YAML::Value << report.rms_px;
    out << YAML::Key << "inlier_ratio" << YAML::Value << report.inlier_ratio;
    out << YAML::Key << "chamfer_px" << YAML::Value << report.chamfer_px;
    out << YAML::Key << "edge_overlap_ratio" << YAML::Value << report.edge_overlap_ratio;
    out << YAML::EndMap;

    out << YAML::Key << "thresholds" << YAML::BeginMap;
    out << YAML::Key << "ncc_good" << YAML::Value << thresholds.ncc_good;
    out << YAML::Key << "ncc_acceptable" << YAML::Value << thresholds.ncc_acceptable;
    out << YAML::Key << "rms_good_px" << YAML::Value << thresholds.rms_good_px;
    out << YAML::Key << "rms_acceptable_px" << YAML::Value << thresholds.rms_acceptable_px;
    out << YAML::Key << "inlier_ratio_good" << YAML::Value << thresholds.inlier_ratio_good;
    out << YAML::Key << "inlier_ratio_acceptable" << YAML::Value << thresholds.inlier_ratio_acceptable;
    out << YAML::Key << "chamfer_good_px" << YAML::Value << thresholds.chamfer_good_px;
    out << YAML::Key << "chamfer_acceptable_px" << YAML::Value << thresholds.chamfer_acceptable_px;
    out << YAML::EndMap;

    out << YAML::Key << "verdict" << YAML::Value << quality_verdict_str(report.verdict);
    out << YAML::Key << "confidence" << YAML::Value << report.confidence;

    if (!report.reasons.empty()) {
        out << YAML::Key << "reasons" << YAML::BeginSeq;
        for (const auto& s : report.reasons) out << s;
        out << YAML::EndSeq;
    }
    if (!report.suggestions.empty()) {
        out << YAML::Key << "suggestions" << YAML::BeginSeq;
        for (const auto& s : report.suggestions) out << s;
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_WARN("无法写入质量报告: {}", path);
        return false;
    }
    f << out.c_str();
    UNICALIB_INFO("[Quality] 质量报告已保存: {}", path);
    return true;
}

bool save_lidar_cam_quality_summary_yaml(
    const std::string& path,
    const std::vector<LidarCamPairQualityEntry>& entries,
    QualityVerdict overall_verdict,
    double min_confidence) {

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "calibration_type" << YAML::Value << "lidar_camera_extrinsic";
    out << YAML::Key << "stage" << YAML::Value << "fine";
    out << YAML::Key << "overall_verdict" << YAML::Value << quality_verdict_str(overall_verdict);
    out << YAML::Key << "min_confidence" << YAML::Value << min_confidence;
    out << YAML::Key << "pair_count" << YAML::Value << static_cast<int>(entries.size());
    out << YAML::Key << "pairs" << YAML::BeginSeq;
    for (const auto& e : entries) {
        out << YAML::BeginMap;
        out << YAML::Key << "lidar_id" << YAML::Value << e.lidar_id;
        out << YAML::Key << "camera_id" << YAML::Value << e.camera_id;
        out << YAML::Key << "method_used" << YAML::Value << e.method_used;
        out << YAML::Key << "verdict" << YAML::Value << quality_verdict_str(e.report.verdict);
        out << YAML::Key << "confidence" << YAML::Value << e.report.confidence;
        out << YAML::Key << "ncc" << YAML::Value << e.report.ncc;
        out << YAML::Key << "rms_px" << YAML::Value << e.report.rms_px;
        out << YAML::Key << "chamfer_px" << YAML::Value << e.report.chamfer_px;
        out << YAML::Key << "edge_overlap_ratio" << YAML::Value << e.report.edge_overlap_ratio;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_WARN("无法写入质量汇总: {}", path);
        return false;
    }
    f << out.c_str();
    UNICALIB_INFO("[Quality] 质量汇总已保存: {} ({} 对, overall={})",
                    path, entries.size(), quality_verdict_str(overall_verdict));
    return true;
}

}  // namespace ns_unicalib
