#include "unicalib/io/new_format_data_source.h"
#include "unicalib/io/lidar_packet_reader.h"
#include "unicalib/io/oem7_imu_reader.h"

#include "unicalib/common/logger.h"

#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace fs = std::filesystem;

namespace ns_unicalib {

namespace {

/**
 * 解析CSV行，按逗号分割
 * @param line CSV行字符串
 * @return 分割后的字符串向量
 */
std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
        out.push_back(token);
    }
    return out;
}

/**
 * 解析文件路径，支持相对路径和绝对路径
 * @param root 根目录
 * @param p 原始路径
 * @return 解析后的绝对路径
 */
std::string resolve_path(const fs::path& root, const std::string& p) {
    if (p.empty()) return p;
    fs::path raw(p);
    if (raw.is_absolute()) return raw.string();
    return (root / raw).lexically_normal().string();
}

double timestamp_unit_scale_from_string(const std::string& unit) {
    if (unit == "ns") return 1e-9;
    if (unit == "us") return 1e-6;
    if (unit == "ms") return 1e-3;
    return 1.0;
}

Oem7ImuTimeBase oem7_time_base_from_cfg_string(const std::string& s) {
    std::string t = s;
    for (char& c : t) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (t == "unix" || t == "unix_utc" || t == "utc" || t == "unix_sec") {
        return Oem7ImuTimeBase::UnixUtcApprox;
    }
    return Oem7ImuTimeBase::GpsSinceEpoch;
}

Oem7ImuDecodeParams make_oem7_decode_params(const NewFormatConfig& cfg) {
    Oem7ImuDecodeParams p;
    p.imu_output_rate_hz = cfg.oem7_imu_output_rate_hz;
    p.time_base = oem7_time_base_from_cfg_string(cfg.oem7_time_base);
    p.gps_minus_utc_leap_sec = cfg.oem7_gps_utc_leap_sec;
    p.time_offset_sec = cfg.oem7_time_offset_sec;
    return p;
}

}  // namespace

/**
 * 构造函数
 * @param cfg 新格式数据源配置
 */
NewFormatDataSource::NewFormatDataSource(const NewFormatConfig& cfg) : cfg_(cfg) {}

/**
 * 将时间戳转换为秒为单位
 * 支持纳秒、微秒、毫秒和秒四种单位
 * @param ts 原始时间戳
 * @return 转换为秒的时间戳
 */
double NewFormatDataSource::to_seconds(double ts) const {
    if (cfg_.timestamp_unit == "ns") return ts * 1e-9;   // 纳秒转秒
    if (cfg_.timestamp_unit == "us") return ts * 1e-6;   // 微秒转秒
    if (cfg_.timestamp_unit == "ms") return ts * 1e-3;   // 毫秒转秒
    return ts;  // 默认已经是秒
}

/**
 * 判断是否保留当前帧（采样策略）
 * @param ts 当前时间戳
 * @param last_kept_ts 上一保留帧的时间戳
 * @param kept_count 已保留的帧数
 * @return true表示应该保留
 */
bool NewFormatDataSource::should_keep(double ts, double& last_kept_ts, size_t kept_count) const {
    // 检查是否达到最大帧数限制
    if (cfg_.max_frames > 0 && kept_count >= cfg_.max_frames) return false;
    // 检查采样间隔（降采样）
    if (cfg_.sample_interval > 0.0 && last_kept_ts > 0.0 && 
        (ts - last_kept_ts) < cfg_.sample_interval) return false;
    return true;
}

/**
 * 加载所有传感器数据
 * 支持的数据类型：
 * - LiDAR: PCD文件或原始包文件（Livox/Hesai/RS-Helios）
 * - Camera: 图像文件（通过OpenCV读取）
 * - IMU: CSV（7列）或 NovAtel OEM7 短二进制 CORRIMUDATAS（索引 2 列）
 * @return true表示加载成功
 */
bool NewFormatDataSource::load() {
    // 清空现有数据
    lidar_data_.clear();
    camera_data_.clear();
    imu_data_.clear();
    loaded_ = false;

    // 验证根目录配置
    if (cfg_.root_dir.empty()) {
        status_msg_ = "new_format.root_dir 为空";
        return false;
    }

    fs::path root(cfg_.root_dir);
    if (!fs::exists(root)) {
        status_msg_ = "new_format.root_dir 不存在: " + cfg_.root_dir;
        return false;
    }

    try {
        // ========== 加载LiDAR数据 ==========
        // 支持两种索引格式：
        // 1) timestamp,path                    （单文件，自动识别格式）
        // 2) timestamp,path_msop,path_difop   （RS雷达显式配对）
        for (const auto& [sensor_id, idx_rel] : cfg_.lidar_index_files) {
            std::string idx_path = resolve_path(root, idx_rel);
            std::ifstream ifs(idx_path);
            if (!ifs.is_open()) {
                UNICALIB_WARN("[NewFormatDataSource] LiDAR 索引打开失败: {}", idx_path);
                continue;
            }
            
            std::string line;
            size_t kept = 0;
            double last_kept_ts = -1.0;
            
            size_t line_no = 0;
            while (std::getline(ifs, line)) {
                ++line_no;
                // 跳过空行和注释行
                if (line.empty() || line[0] == '#') continue;
                auto cols = split_csv_line(line);
                if (cols.size() < 2) {
                    UNICALIB_WARN("[NewFormatDataSource] LiDAR CSV 列数不足: sensor={} file={} line={} content={}",
                                  sensor_id, idx_path, line_no, line);
                    continue;
                }

                double ts = 0.0;
                try {
                    // 解析时间戳
                    const double ts_raw = std::stod(cols[0]);
                    ts = to_seconds(ts_raw);
                } catch (const std::exception& e) {
                    UNICALIB_WARN("[NewFormatDataSource] LiDAR 时间戳解析失败: sensor={} file={} line={} err={} content={}",
                                  sensor_id, idx_path, line_no, e.what(), line);
                    continue;
                }
                if (!should_keep(ts, last_kept_ts, kept)) continue;

                // 第二列是主数据文件路径
                std::string pcd_path = resolve_path(root, cols[1]);
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                bool loaded = (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == 0);
                
                if (loaded) {
                    // 成功加载PCD文件
                    lidar_data_[sensor_id].emplace_back(ts, cloud);
                } else {
                    // 非PCD文件，尝试作为原始数据包解码
                    std::vector<DecodedLidarFrame> decoded;
                    bool dec_ok = false;
                    
                    if (cols.size() >= 3 && !cols[2].empty()) {
                        // 提供了DIFOP文件，按RS格式解码
                        const std::string difop_path = resolve_path(root, cols[2]);
                        dec_ok = LidarPacketReader::decode_rs_msop_difop(pcd_path, difop_path, decoded, 1);
                    } else {
                        // 自动识别格式解码
                        dec_ok = LidarPacketReader::decode_file(
                            pcd_path, decoded, 1, timestamp_unit_scale_from_string(cfg_.timestamp_unit));
                    }
                    
                    if (dec_ok && !decoded.empty() && decoded[0].cloud) {
                        // 优先使用索引文件的时间戳，无效时使用解码器的时间戳
                        const double final_ts = (ts > 0.0) ? ts : decoded[0].timestamp_sec;
                        lidar_data_[sensor_id].emplace_back(final_ts, decoded[0].cloud);
                    } else {
                        UNICALIB_WARN("[NewFormatDataSource] LiDAR 解码失败: sensor={} file={} line={} data={}",
                                      sensor_id, idx_path, line_no, pcd_path);
                        continue;  // 解码失败，跳过
                    }
                }
                last_kept_ts = (ts > 0.0) ? ts : last_kept_ts;
                ++kept;
            }
            UNICALIB_INFO("[NewFormatDataSource] LiDAR {} 加载 {} 帧", sensor_id, lidar_data_[sensor_id].size());
        }

        // ========== 加载相机数据 ==========
        // 索引格式：timestamp,image_path
        for (const auto& [sensor_id, idx_rel] : cfg_.camera_index_files) {
            std::string idx_path = resolve_path(root, idx_rel);
            std::ifstream ifs(idx_path);
            if (!ifs.is_open()) {
                UNICALIB_WARN("[NewFormatDataSource] 相机索引打开失败: {}", idx_path);
                continue;
            }
            
            std::string line;
            size_t kept = 0;
            double last_kept_ts = -1.0;
            
            size_t line_no = 0;
            while (std::getline(ifs, line)) {
                ++line_no;
                if (line.empty() || line[0] == '#') continue;
                auto cols = split_csv_line(line);
                if (cols.size() < 2) {
                    UNICALIB_WARN("[NewFormatDataSource] Camera CSV 列数不足: sensor={} file={} line={} content={}",
                                  sensor_id, idx_path, line_no, line);
                    continue;
                }

                double ts = 0.0;
                try {
                    const double ts_raw = std::stod(cols[0]);
                    ts = to_seconds(ts_raw);
                } catch (const std::exception& e) {
                    UNICALIB_WARN("[NewFormatDataSource] Camera 时间戳解析失败: sensor={} file={} line={} err={} content={}",
                                  sensor_id, idx_path, line_no, e.what(), line);
                    continue;
                }
                if (!should_keep(ts, last_kept_ts, kept)) continue;

                // 读取图像
                std::string img_path = resolve_path(root, cols[1]);
                cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
                if (image.empty()) {
                    UNICALIB_WARN("[NewFormatDataSource] Camera 图像读取失败: sensor={} file={} line={} image={}",
                                  sensor_id, idx_path, line_no, img_path);
                    continue;  // 图像读取失败，跳过
                }
                
                camera_data_[sensor_id].emplace_back(ts, image);
                last_kept_ts = ts;
                ++kept;
            }
            UNICALIB_INFO("[NewFormatDataSource] 相机 {} 加载 {} 帧", sensor_id, camera_data_[sensor_id].size());
        }

        // ========== 加载IMU数据 ==========
        // 索引格式 A：timestamp,gx,gy,gz,ax,ay,az（陀螺 rad/s，加速度 m/s²）
        // 索引格式 B：占位,timestamp_ignored,oem7_short_binary_path（两列：与 LiDAR 索引类似，首列可填 0）
        for (const auto& [sensor_id, idx_rel] : cfg_.imu_index_files) {
            std::string idx_path = resolve_path(root, idx_rel);
            std::ifstream ifs(idx_path);
            if (!ifs.is_open()) {
                UNICALIB_WARN("[NewFormatDataSource] IMU 索引打开失败: {}", idx_path);
                continue;
            }

            std::string line;
            size_t kept = 0;
            double last_kept_ts = -1.0;
            std::set<std::string> oem7_paths_emitted;

            size_t line_no = 0;
            while (std::getline(ifs, line)) {
                ++line_no;
                if (line.empty() || line[0] == '#') continue;
                auto cols = split_csv_line(line);

                if (cols.size() == 2) {
                    const std::string bin_path = resolve_path(root, cols[1]);
                    if (bin_path.empty()) {
                        UNICALIB_WARN("[NewFormatDataSource] IMU OEM7 路径为空: sensor={} file={} line={}",
                                      sensor_id, idx_path, line_no);
                        continue;
                    }
                    if (oem7_paths_emitted.count(bin_path) != 0u) {
                        UNICALIB_WARN("[NewFormatDataSource] IMU 索引重复引用同一 OEM7 文件，已跳过: sensor={} path={}",
                                      sensor_id, bin_path);
                        continue;
                    }
                    std::vector<IMUFrameRos> decoded;
                    std::string err;
                    if (!decode_oem7_imu_binary_file(bin_path, make_oem7_decode_params(cfg_), decoded, err)) {
                        UNICALIB_WARN("[NewFormatDataSource] IMU OEM7 解码失败: sensor={} err={}", sensor_id, err);
                        continue;
                    }
                    oem7_paths_emitted.insert(bin_path);
                    for (const IMUFrameRos& fr : decoded) {
                        if (!should_keep(fr.timestamp, last_kept_ts, kept)) continue;
                        imu_data_[sensor_id].push_back(fr);
                        last_kept_ts = fr.timestamp;
                        ++kept;
                    }
                    continue;
                }

                if (cols.size() < 7) {
                    UNICALIB_WARN("[NewFormatDataSource] IMU 索引列数无效（需 7 列 CSV 或 2 列 OEM7 路径）: sensor={} "
                                  "file={} line={} content={}",
                                  sensor_id, idx_path, line_no, line);
                    continue;
                }

                double ts = 0.0;
                try {
                    const double ts_raw = std::stod(cols[0]);
                    ts = to_seconds(ts_raw);
                } catch (const std::exception& e) {
                    UNICALIB_WARN("[NewFormatDataSource] IMU 时间戳解析失败: sensor={} file={} line={} err={} content={}",
                                  sensor_id, idx_path, line_no, e.what(), line);
                    continue;
                }
                if (!should_keep(ts, last_kept_ts, kept)) continue;

                IMUFrameRos frame;
                try {
                    frame.timestamp = ts;
                    frame.gyro[0] = std::stod(cols[1]);
                    frame.gyro[1] = std::stod(cols[2]);
                    frame.gyro[2] = std::stod(cols[3]);
                    frame.accel[0] = std::stod(cols[4]);
                    frame.accel[1] = std::stod(cols[5]);
                    frame.accel[2] = std::stod(cols[6]);
                } catch (const std::exception& e) {
                    UNICALIB_WARN("[NewFormatDataSource] IMU 数据解析失败: sensor={} file={} line={} err={} content={}",
                                  sensor_id, idx_path, line_no, e.what(), line);
                    continue;
                }
                imu_data_[sensor_id].push_back(frame);
                last_kept_ts = ts;
                ++kept;
            }
            UNICALIB_INFO("[NewFormatDataSource] IMU {} 加载 {} 帧", sensor_id, imu_data_[sensor_id].size());
        }

        loaded_ = true;
        status_msg_ = "new_format 数据加载成功";
        return true;
    } catch (const std::exception& e) {
        status_msg_ = std::string("new_format 加载失败: ") + e.what();
        UNICALIB_ERROR("[NewFormatDataSource] {}", status_msg_);
        return false;
    }
}

/**
 * 获取指定ID的LiDAR扫描数据
 * @param sensor_id 传感器ID
 * @return 扫描数据列表
 */
std::vector<LiDARScanRos> NewFormatDataSource::get_lidar_scans(const std::string& sensor_id) const {
    auto it = lidar_data_.find(sensor_id);
    return it == lidar_data_.end() ? std::vector<LiDARScanRos>{} : it->second;
}

/**
 * 获取指定ID的相机帧数据
 * @param sensor_id 传感器ID
 * @return 相机帧数据列表
 */
std::vector<CameraFrameRos> NewFormatDataSource::get_camera_frames(const std::string& sensor_id) const {
    auto it = camera_data_.find(sensor_id);
    return it == camera_data_.end() ? std::vector<CameraFrameRos>{} : it->second;
}

/**
 * 获取指定ID的IMU帧数据
 * @param sensor_id 传感器ID
 * @return IMU帧数据列表
 */
std::vector<IMUFrameRos> NewFormatDataSource::get_imu_frames(const std::string& sensor_id) const {
    auto it = imu_data_.find(sensor_id);
    return it == imu_data_.end() ? std::vector<IMUFrameRos>{} : it->second;
}

/**
 * 获取所有LiDAR传感器ID
 * @return 传感器ID列表
 */
std::vector<std::string> NewFormatDataSource::get_lidar_ids() const {
    std::vector<std::string> out;
    out.reserve(lidar_data_.size());
    for (const auto& [id, _] : lidar_data_) out.push_back(id);
    return out;
}

/**
 * 获取所有相机传感器ID
 * @return 传感器ID列表
 */
std::vector<std::string> NewFormatDataSource::get_camera_ids() const {
    std::vector<std::string> out;
    out.reserve(camera_data_.size());
    for (const auto& [id, _] : camera_data_) out.push_back(id);
    return out;
}

/**
 * 获取所有IMU传感器ID
 * @return 传感器ID列表
 */
std::vector<std::string> NewFormatDataSource::get_imu_ids() const {
    std::vector<std::string> out;
    out.reserve(imu_data_.size());
    for (const auto& [id, _] : imu_data_) out.push_back(id);
    return out;
}

}  // namespace ns_unicalib