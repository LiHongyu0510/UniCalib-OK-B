#include "unicalib/extrinsic/lidar_scan_pair.h"
#include "unicalib/common/logger.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>

namespace ns_unicalib {

namespace {

double normalize_epoch_seconds(double t) {
    if (!std::isfinite(t)) return t;
    // 文件名常见毫秒 epoch（13 位）
    if (std::fabs(t) > 1e11) return t * 1e-3;
    // 微秒（16 位）
    if (std::fabs(t) > 1e14) return t * 1e-6;
    return t;
}

bool lidar_scan_valid(const LiDARScan& s) {
    return s.cloud && !s.cloud->empty();
}

}  // namespace

std::optional<double> parse_timestamp_from_filename_stem(const std::string& stem) {
    if (stem.empty()) return std::nullopt;
    try {
        size_t pos = 0;
        const double v = std::stod(stem, &pos);
        if (pos > 0) return normalize_epoch_seconds(v);
    } catch (...) {
    }
    static const std::regex k_leading_num(R"(^(\d+(?:\.\d+)?))");
    std::smatch m;
    if (std::regex_search(stem, m, k_leading_num) && m.size() >= 2) {
        try {
            return normalize_epoch_seconds(std::stod(m[1].str()));
        } catch (...) {
        }
    }
    return std::nullopt;
}

std::pair<std::size_t, double> nearest_lidar_scan_index_for_camera_ts(
    const std::vector<LiDARScan>& lidar_scans,
    double ts_cam,
    double time_offset_s) {
    const double target_ts = ts_cam + time_offset_s;
    std::size_t best_idx = 0;
    double best_dt = std::numeric_limits<double>::infinity();
    bool any = false;
    for (std::size_t i = 0; i < lidar_scans.size(); ++i) {
        if (!lidar_scan_valid(lidar_scans[i])) continue;
        const double dt = std::fabs(lidar_scans[i].timestamp - target_ts);
        if (!any || dt < best_dt) {
            best_dt = dt;
            best_idx = i;
            any = true;
        }
    }
    if (!any) return {0, best_dt};
    return {best_idx, best_dt};
}

std::pair<std::size_t, std::size_t> time_aligned_lidar_scan_indices_for_manual(
    const std::vector<LiDARScan>& ref,
    const std::vector<LiDARScan>& tgt) {
    if (ref.empty() || tgt.empty())
        return {0, 0};
    double best_dt = 1e300;
    std::size_t best_i = 0, best_j = 0;
    bool any = false;
    for (std::size_t i = 0; i < ref.size(); ++i) {
        if (!lidar_scan_valid(ref[i])) continue;
        for (std::size_t j = 0; j < tgt.size(); ++j) {
            if (!lidar_scan_valid(tgt[j])) continue;
            const double dt = std::fabs(ref[i].timestamp - tgt[j].timestamp);
            if (!any || dt < best_dt) {
                best_dt = dt;
                best_i = i;
                best_j = j;
                any = true;
            }
        }
    }
    if (!any) return {0, 0};
    return {best_i, best_j};
}

std::vector<LidarCamSyncIndexPair> collect_lidar_cam_sync_index_pairs(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    double sync_threshold_s,
    double time_offset_s) {
    std::vector<std::size_t> cam_order(camera_frames.size());
    for (std::size_t i = 0; i < camera_frames.size(); ++i) cam_order[i] = i;
    std::sort(cam_order.begin(), cam_order.end(),
              [&](std::size_t a, std::size_t b) {
                  return camera_frames[a].first < camera_frames[b].first;
              });

    std::vector<LidarCamSyncIndexPair> pairs;
    pairs.reserve(cam_order.size());
    for (std::size_t ci : cam_order) {
        if (camera_frames[ci].second.empty()) continue;
        auto [li, dt] = nearest_lidar_scan_index_for_camera_ts(
            lidar_scans, camera_frames[ci].first, time_offset_s);
        if (!std::isfinite(dt) || dt > sync_threshold_s) continue;
        if (!lidar_scan_valid(lidar_scans[li])) continue;
        pairs.push_back({ci, li, dt});
    }
    return pairs;
}

std::pair<std::size_t, std::size_t> time_aligned_lidar_cam_indices_for_manual(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    double sync_threshold_s,
    double time_offset_s) {
    const auto pairs = collect_lidar_cam_sync_index_pairs(
        lidar_scans, camera_frames, sync_threshold_s, time_offset_s);
    if (pairs.empty()) {
        if (!camera_frames.empty() && !lidar_scans.empty())
            return {0, 0};
        return {0, 0};
    }
    const auto best = std::min_element(
        pairs.begin(), pairs.end(),
        [](const LidarCamSyncIndexPair& a, const LidarCamSyncIndexPair& b) {
            return a.dt_s < b.dt_s;
        });
    return {best->cam_idx, best->lidar_idx};
}

std::size_t build_time_aligned_lidar_camera_sequences(
    std::vector<LiDARScan>& lidar_out,
    std::vector<std::pair<double, cv::Mat>>& camera_out,
    const std::vector<LiDARScan>& lidar_in,
    const std::vector<std::pair<double, cv::Mat>>& camera_in,
    double sync_threshold_s,
    double time_offset_s,
    std::size_t max_pairs) {
    lidar_out.clear();
    camera_out.clear();
    if (lidar_in.empty() || camera_in.empty() || max_pairs == 0)
        return 0;

    auto pairs = collect_lidar_cam_sync_index_pairs(
        lidar_in, camera_in, sync_threshold_s, time_offset_s);
    if (pairs.empty()) return 0;

    if (pairs.size() > max_pairs) {
        const double step = static_cast<double>(pairs.size()) / static_cast<double>(max_pairs);
        std::vector<LidarCamSyncIndexPair> subsampled;
        subsampled.reserve(max_pairs);
        for (std::size_t k = 0; k < max_pairs; ++k) {
            const std::size_t idx = static_cast<std::size_t>(std::floor(k * step));
            if (idx < pairs.size()) subsampled.push_back(pairs[idx]);
        }
        pairs = std::move(subsampled);
    }

    lidar_out.reserve(pairs.size());
    camera_out.reserve(pairs.size());
    double dt_sum = 0.0, dt_max = 0.0;
    for (const auto& p : pairs) {
        lidar_out.push_back(lidar_in[p.lidar_idx]);
        camera_out.emplace_back(camera_in[p.cam_idx].first, camera_in[p.cam_idx].second);
        dt_sum += p.dt_s;
        dt_max = std::max(dt_max, p.dt_s);
    }
    const double dt_mean = dt_sum / static_cast<double>(pairs.size());
    UNICALIB_INFO(
        "[LiDAR-Cam] 时间戳自动匹配: 相机 {} 帧 / LiDAR {} 帧 -> 有效对 {} 对 "
        "(阈值 {:.3f}s, |dt| mean={:.4f}s max={:.4f}s)",
        camera_in.size(), lidar_in.size(), pairs.size(),
        sync_threshold_s, dt_mean, dt_max);
    return pairs.size();
}

}  // namespace ns_unicalib
