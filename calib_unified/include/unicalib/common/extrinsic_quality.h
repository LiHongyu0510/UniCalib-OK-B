#pragma once
/**
 * 外参标定质量评估 — 统一 PASS/FAIL 判定与 YAML 报告输出
 * 与 EXTRINSIC_QUALITY_SCHEMA.md / quality_assessment.py 语义对齐
 */

#include <string>
#include <vector>

namespace ns_unicalib {

enum class QualityVerdict {
    GOOD,
    ACCEPTABLE,
    BAD,
    UNKNOWN,
};

inline const char* quality_verdict_str(QualityVerdict v) {
    switch (v) {
        case QualityVerdict::GOOD:       return "good";
        case QualityVerdict::ACCEPTABLE: return "acceptable";
        case QualityVerdict::BAD:        return "bad";
        default:                         return "unknown";
    }
}

struct QualityThresholds {
    double ncc_good = 0.3;
    double ncc_acceptable = 0.2;
    double rms_good_px = 2.0;
    double rms_acceptable_px = 5.0;
    double inlier_ratio_good = 0.6;
    double inlier_ratio_acceptable = 0.4;
    double chamfer_good_px = 3.0;
    double chamfer_acceptable_px = 6.0;
};

struct ExtrinsicQualityReport {
    QualityVerdict verdict = QualityVerdict::UNKNOWN;
    double confidence = 0.0;
    double ncc = -1.0;
    double rms_px = -1.0;
    double inlier_ratio = -1.0;
    double chamfer_px = -1.0;
    double edge_overlap_ratio = -1.0;
    std::vector<std::string> reasons;
    std::vector<std::string> suggestions;
};

struct LidarCamPairQualityEntry {
    std::string lidar_id;
    std::string camera_id;
    std::string method_used;
    ExtrinsicQualityReport report;
};

ExtrinsicQualityReport assess_lidar_cam_quality(
    double ncc,
    double rms_px,
    double inlier_ratio = -1.0,
    double chamfer_px = -1.0,
    double edge_overlap_ratio = -1.0,
    const QualityThresholds& thresholds = {});

bool save_quality_report_yaml(
    const std::string& path,
    const std::string& calibration_type,
    const std::string& stage,
    const std::string& reference_sensor,
    const std::string& target_sensor,
    const std::string& method_used,
    bool has_valid_extrinsic,
    const ExtrinsicQualityReport& report,
    const QualityThresholds& thresholds);

bool save_lidar_cam_quality_summary_yaml(
    const std::string& path,
    const std::vector<LidarCamPairQualityEntry>& entries,
    QualityVerdict overall_verdict,
    double min_confidence);

}  // namespace ns_unicalib
