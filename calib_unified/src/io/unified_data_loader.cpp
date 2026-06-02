/**
 * UnifiedDataLoader — 文件模式 + NEW_FORMAT（不依赖 ROS2 运行时库）
 * ROS2 bag/话题 的 load_from_ros 在 ros2_data_source.cpp（仅 UNICALIB_WITH_ROS2 时编译）
 */

#include "unicalib/io/ros2_data_source.h"
#include "unicalib/io/new_format_data_source.h"
#include "unicalib/io/new_format_path.h"
#include "unicalib/common/logger.h"
#include "unicalib/io/yaml_io.h"

#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace fs = std::filesystem;

namespace ns_unicalib {

namespace {

double parse_pcd_timestamp_from_stem(const std::string& stem) {
    auto pos = stem.rfind('_');
    std::string ts_part = (pos != std::string::npos) ? stem.substr(pos + 1) : stem;
    try {
        double v = std::stod(ts_part);
        if (v > 1e18) return v / 1e9;
        if (v > 1e15) return v / 1e6;
        if (v > 1e12) return v / 1e3;
        if (v > 1e9) return v;
        return v;
    } catch (...) {
        return -1.0;
    }
}

std::string resolve_imu_csv_path(const std::string& path) {
    if (path.empty() || !fs::exists(path)) return "";
    if (fs::is_regular_file(path)) return path;
    if (!fs::is_directory(path)) return "";
    std::string best;
    for (const auto& entry : fs::directory_iterator(path)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext != ".csv") continue;
        std::string name = entry.path().filename().string();
        if (name.find("index") != std::string::npos) continue;
        std::string lower = name;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        if (lower == "imu.csv") return entry.path().string();
        if (best.empty()) best = entry.path().string();
    }
    return best;
}

bool load_lidar_pcd_dir(const std::string& sensor_id, const std::string& dir,
                        size_t max_frames, double sample_interval,
                        std::map<std::string, std::vector<LiDARScanRos>>& out) {
    if (dir.empty() || !fs::exists(dir) || !fs::is_directory(dir)) {
        UNICALIB_WARN("[UnifiedDataLoader] LiDAR 目录无效: {} ({})", sensor_id, dir);
        return false;
    }
    std::vector<fs::path> pcd_files;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".pcd") pcd_files.push_back(entry.path());
    }
    std::sort(pcd_files.begin(), pcd_files.end());
    auto& scans = out[sensor_id];
    size_t loaded = 0;
    double last_ts = -1e30;
    for (const auto& pcd_path : pcd_files) {
        if (max_frames > 0 && loaded >= max_frames) break;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path.string(), *cloud) != 0) continue;
        double ts = parse_pcd_timestamp_from_stem(pcd_path.stem().string());
        if (ts < 0.0) ts = static_cast<double>(scans.size()) * 0.1;
        if (sample_interval > 0.0 && !scans.empty() && (ts - last_ts) < sample_interval) continue;
        scans.push_back(LiDARScanRos(ts, cloud));
        last_ts = ts;
        ++loaded;
    }
    UNICALIB_INFO("  [{}] 加载 {} 帧 LiDAR 点云 ({})", sensor_id, scans.size(), dir);
    return !scans.empty();
}

}  // namespace

UnifiedDataLoader::UnifiedDataLoader(const Config& cfg) : cfg_(cfg) {
    status_msg_ = "未加载";
}

UnifiedDataLoader::~UnifiedDataLoader() = default;

bool UnifiedDataLoader::load_from_files() {
    status_msg_ = "从文件加载数据...";

    UNICALIB_INFO("[UnifiedDataLoader] 文件模式: LiDAR 传感器 {} 个, IMU 传感器 {} 个",
                  cfg_.file_lidar_dirs.size() + (cfg_.lidar_data_dir.empty() ? 0u : 1u),
                  cfg_.file_imu_paths.size());

    try {
        if (!cfg_.file_lidar_dirs.empty()) {
            for (const auto& [sid, dir] : cfg_.file_lidar_dirs)
                load_lidar_pcd_dir(sid, dir, cfg_.max_frames, cfg_.sample_interval, file_lidar_data_);
        } else if (!cfg_.lidar_data_dir.empty()) {
            load_lidar_pcd_dir("lidar_front", cfg_.lidar_data_dir, cfg_.max_frames,
                               cfg_.sample_interval, file_lidar_data_);
        }

        for (const auto& [sid, imu_path] : cfg_.file_imu_paths) {
            std::string csv = resolve_imu_csv_path(imu_path);
            if (csv.empty()) {
                UNICALIB_WARN("[UnifiedDataLoader] 未找到 IMU CSV: {} ({})", sid, imu_path);
                continue;
            }
            IMURawData raw = YamlIO::load_imu_csv(csv);
            if (cfg_.max_frames > 0 && raw.size() > cfg_.max_frames)
                raw.resize(cfg_.max_frames);
            file_imu_raw_data_[sid] = std::move(raw);
            UNICALIB_INFO("  [{}] IMU {} 帧 ({})", sid, file_imu_raw_data_[sid].size(), csv);
        }

        if (!cfg_.camera_images_dir.empty() && fs::exists(cfg_.camera_images_dir)) {
            std::vector<fs::path> img_files;
            for (const auto& entry : fs::directory_iterator(cfg_.camera_images_dir)) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                    img_files.push_back(entry.path());
                }
            }
            std::sort(img_files.begin(), img_files.end());

            for (const auto& img_path : img_files) {
                cv::Mat img = cv::imread(img_path.string());
                if (!img.empty()) {
                    double ts;
                    try {
                        ts = std::stod(img_path.stem().string());
                    } catch (...) {
                        ts = static_cast<double>(file_camera_data_["cam_left"].size()) * 0.1;
                    }
                    file_camera_data_["cam_left"].push_back(CameraFrameRos(ts, img));
                }
            }
            UNICALIB_INFO("  加载 {} 帧相机图像", file_camera_data_["cam_left"].size());
        }

        if (!cfg_.camera_intrinsic_file.empty() && fs::exists(cfg_.camera_intrinsic_file)) {
            try {
                auto intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
                camera_intrinsics_["cam_left"] = intrin;
                UNICALIB_INFO("  加载相机内参: fx={:.1f} fy={:.1f}", intrin.fx, intrin.fy);
            } catch (const std::exception& e) {
                UNICALIB_WARN("内参加载失败: {}", e.what());
            }
        }

        loaded_ = true;
        status_msg_ = "文件加载成功";
        return true;

    } catch (const std::exception& e) {
        status_msg_ = "文件加载失败: " + std::string(e.what());
        UNICALIB_ERROR("[UnifiedDataLoader] {}", status_msg_);
        return false;
    }
}

bool UnifiedDataLoader::load_from_new_format() {
    status_msg_ = "从新采集格式加载数据...";
    NewFormatConfig cfg;
    cfg.root_dir = resolve_new_format_root_dir(cfg_.new_format_root_dir);
    cfg.lidar_index_files = cfg_.new_format_lidar_index_files;
    cfg.camera_index_files = cfg_.new_format_camera_index_files;
    cfg.imu_index_files = cfg_.new_format_imu_index_files;
    cfg.timestamp_unit = cfg_.new_format_timestamp_unit;
    cfg.oem7_imu_output_rate_hz = cfg_.new_format_oem7_imu_rate_hz;
    cfg.oem7_time_base = cfg_.new_format_oem7_time_base;
    cfg.oem7_gps_utc_leap_sec = cfg_.new_format_oem7_gps_utc_leap_sec;
    cfg.oem7_time_offset_sec = cfg_.new_format_oem7_time_offset_sec;
    cfg.oem7_imu_gyro_scale_factor = cfg_.new_format_oem7_gyro_scale_factor;
    cfg.oem7_imu_accel_scale_factor = cfg_.new_format_oem7_accel_scale_factor;
    cfg.max_frames = cfg_.max_frames;
    cfg.sample_interval = cfg_.sample_interval;

    UNICALIB_INFO("[UnifiedDataLoader] 从新采集格式加载数据...");
    UNICALIB_INFO("  root_dir: {}", cfg.root_dir);
    UNICALIB_INFO("  timestamp_unit: {}", cfg.timestamp_unit);
    UNICALIB_INFO("  oem7: rate={} Hz time_base={} leap={} offset={} s gyro_scale={} accel_scale={}",
                  cfg.oem7_imu_output_rate_hz, cfg.oem7_time_base, cfg.oem7_gps_utc_leap_sec,
                  cfg.oem7_time_offset_sec, cfg.oem7_imu_gyro_scale_factor, cfg.oem7_imu_accel_scale_factor);

    new_format_source_ = std::make_shared<NewFormatDataSource>(cfg);
    if (!new_format_source_->load()) {
        status_msg_ = new_format_source_->get_status_message();
        return false;
    }

    loaded_ = true;
    status_msg_ = "新采集格式数据加载成功";
    return true;
}

#if !defined(UNICALIB_WITH_ROS2) || !UNICALIB_WITH_ROS2
bool UnifiedDataLoader::load_from_ros() {
    status_msg_ = "当前构建未启用 ROS2（缺少 rclcpp/rosbag2），无法从 bag 或话题加载";
    UNICALIB_ERROR("[UnifiedDataLoader] {}", status_msg_);
    return false;
}
#endif

bool UnifiedDataLoader::load() {
    if (cfg_.source_type == SourceType::FILES) {
        return load_from_files();
    }
    if (cfg_.source_type == SourceType::ROS2_BAG || cfg_.source_type == SourceType::ROS2_TOPIC) {
        return load_from_ros();
    }
    if (cfg_.source_type == SourceType::NEW_FORMAT) {
        return load_from_new_format();
    }
    status_msg_ = "未知的数据源类型";
    return false;
}

std::vector<LiDARScanRos> UnifiedDataLoader::get_lidar_scans(const std::string& sensor_id) const {
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_lidar_data_.find(sensor_id);
        return (it != file_lidar_data_.end()) ? it->second : std::vector<LiDARScanRos>();
    }
    if (cfg_.source_type == SourceType::NEW_FORMAT) {
        return new_format_source_ ? new_format_source_->get_lidar_scans(sensor_id) : std::vector<LiDARScanRos>();
    }
    return ros_source_ ? ros_source_->get_lidar_scans(sensor_id) : std::vector<LiDARScanRos>();
}

std::vector<CameraFrameRos> UnifiedDataLoader::get_camera_frames(const std::string& sensor_id) const {
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_camera_data_.find(sensor_id);
        return (it != file_camera_data_.end()) ? it->second : std::vector<CameraFrameRos>();
    }
    if (cfg_.source_type == SourceType::NEW_FORMAT) {
        return new_format_source_ ? new_format_source_->get_camera_frames(sensor_id)
                                  : std::vector<CameraFrameRos>();
    }
    return ros_source_ ? ros_source_->get_camera_frames(sensor_id) : std::vector<CameraFrameRos>();
}

std::vector<IMUFrameRos> UnifiedDataLoader::get_imu_frames(const std::string& sensor_id) const {
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_imu_raw_data_.find(sensor_id);
        if (it == file_imu_raw_data_.end()) return {};
        std::vector<IMUFrameRos> frames;
        frames.reserve(it->second.size());
        for (const auto& f : it->second) {
            IMUFrameRos ros_f;
            ros_f.timestamp = f.timestamp;
            ros_f.gyro[0] = f.gyro[0];
            ros_f.gyro[1] = f.gyro[1];
            ros_f.gyro[2] = f.gyro[2];
            ros_f.accel[0] = f.accel[0];
            ros_f.accel[1] = f.accel[1];
            ros_f.accel[2] = f.accel[2];
            frames.push_back(ros_f);
        }
        return frames;
    }
    if (cfg_.source_type == SourceType::NEW_FORMAT) {
        return new_format_source_ ? new_format_source_->get_imu_frames(sensor_id) : std::vector<IMUFrameRos>();
    }
    return ros_source_ ? ros_source_->get_imu_frames(sensor_id) : std::vector<IMUFrameRos>();
}

std::vector<LiDARScan> UnifiedDataLoader::to_lidar_scans(const std::string& sensor_id) const {
    std::vector<LiDARScan> scans;
    auto ros_scans = get_lidar_scans(sensor_id);
    for (const auto& ros_scan : ros_scans) {
        LiDARScan scan;
        scan.timestamp = ros_scan.timestamp;
        scan.cloud = ros_scan.cloud;
        scans.push_back(scan);
    }
    return scans;
}

std::vector<std::pair<double, cv::Mat>> UnifiedDataLoader::to_camera_frames(const std::string& sensor_id) const {
    std::vector<std::pair<double, cv::Mat>> frames;
    auto ros_frames = get_camera_frames(sensor_id);
    for (const auto& ros_frame : ros_frames) {
        frames.emplace_back(ros_frame.timestamp, ros_frame.image);
    }
    return frames;
}

IMURawData UnifiedDataLoader::to_imu_raw_data(const std::string& sensor_id) const {
    IMURawData data;
    auto ros_frames = get_imu_frames(sensor_id);
    for (const auto& ros_frame : ros_frames) {
        IMURawFrame frame;
        frame.timestamp = ros_frame.timestamp;
        frame.gyro[0] = ros_frame.gyro[0];
        frame.gyro[1] = ros_frame.gyro[1];
        frame.gyro[2] = ros_frame.gyro[2];
        frame.accel[0] = ros_frame.accel[0];
        frame.accel[1] = ros_frame.accel[1];
        frame.accel[2] = ros_frame.accel[2];
        data.push_back(frame);
    }
    return data;
}

std::optional<CameraIntrinsics> UnifiedDataLoader::get_camera_intrinsics(const std::string& sensor_id) const {
    auto it = camera_intrinsics_.find(sensor_id);
    if (it != camera_intrinsics_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<std::string> UnifiedDataLoader::get_camera_ids() const {
    if (cfg_.source_type == SourceType::FILES) {
        std::vector<std::string> ids;
        for (const auto& [k, _] : file_camera_data_) {
            ids.push_back(k);
        }
        return ids;
    }
    if (cfg_.source_type == SourceType::NEW_FORMAT && new_format_source_) {
        return new_format_source_->get_camera_ids();
    }
    if (ros_source_) {
        return ros_source_->get_camera_ids();
    }
    return {};
}

bool UnifiedDataLoader::is_ready() const {
    return loaded_;
}

std::string UnifiedDataLoader::get_status_message() const {
    return status_msg_;
}

}  // namespace ns_unicalib
