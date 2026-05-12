/**
 * UniCalib — 标定精度 CSV 记录实现
 */

#include "unicalib/common/accuracy_logger.h"
#include "unicalib/common/logger.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

namespace ns_unicalib {

namespace {

std::string now_iso() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

/** 对 CSV 字段转义：若含逗号/引号/换行则用双引号包裹并转义内部双引号 */
std::string csv_escape(const std::string& s) {
    bool need_quote = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') {
            need_quote = true;
            break;
        }
    }
    if (!need_quote) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out += c;
    }
    out += "\"";
    return out;
}

std::string csv_path(const std::string& output_dir, CalibAccuracyTask task) {
    std::string suffix = accuracy_task_to_suffix(task);
    return (fs::path(output_dir) / ("calib_accuracy_" + suffix + ".csv")).string();
}

/** 每种标定类型的扩展列（固定顺序，保证同一 CSV 列一致） */
std::vector<std::string> extra_columns_for_task(CalibAccuracyTask task) {
    switch (task) {
        case CalibAccuracyTask::CAM_INTRINSIC:
            return {"num_images"};
        case CalibAccuracyTask::IMU_INTRINSIC:
            return {"noise_gyro", "bias_instab_gyro", "noise_accel", "bias_instab_accel"};
        case CalibAccuracyTask::LIDAR_CAM_EXTRIN:
        case CalibAccuracyTask::CAM_CAM_EXTRIN:
        case CalibAccuracyTask::IMU_LIDAR_EXTRIN:
            return {"message"};
        default:
            return {};
    }
}

void write_header(std::ofstream& f,
                  const std::vector<std::string>& extra_keys) {
    f << "timestamp,success,residual_rms,elapsed_ms";
    for (const auto& k : extra_keys) f << "," << k;
    f << "\n";
}

void write_row(std::ofstream& f,
               const std::string& timestamp,
               bool success,
               double residual_rms,
               double elapsed_ms,
               const std::map<std::string, std::string>& extra,
               const std::vector<std::string>& extra_key_order) {
    std::ostringstream rs, es;
    rs << std::fixed << std::setprecision(6) << residual_rms;
    es << std::fixed << std::setprecision(2) << elapsed_ms;
    f << csv_escape(timestamp) << ","
      << (success ? "true" : "false") << ","
      << rs.str() << ","
      << es.str();
    for (const auto& k : extra_key_order) {
        auto it = extra.find(k);
        std::string v = (it != extra.end()) ? it->second : "";
        f << "," << csv_escape(v);
    }
    f << "\n";
}

}  // namespace

std::string accuracy_task_to_suffix(CalibAccuracyTask task) {
    switch (task) {
        case CalibAccuracyTask::CAM_INTRINSIC:     return "cam_intrinsic";
        case CalibAccuracyTask::IMU_INTRINSIC:      return "imu_intrinsic";
        case CalibAccuracyTask::LIDAR_CAM_EXTRIN:   return "lidar_cam_extrin";
        case CalibAccuracyTask::CAM_CAM_EXTRIN:     return "cam_cam_extrin";
        case CalibAccuracyTask::IMU_LIDAR_EXTRIN:   return "imu_lidar_extrin";
    }
    return "unknown";
}

void append_calib_accuracy(
    const std::string& output_dir,
    CalibAccuracyTask task,
    bool success,
    double residual_rms,
    double elapsed_ms,
    const std::map<std::string, std::string>& extra) {

    fs::create_directories(output_dir);
    std::string path = csv_path(output_dir, task);

    std::vector<std::string> extra_keys = extra_columns_for_task(task);
    std::string timestamp = now_iso();
    bool file_existed = fs::exists(path);

    std::ofstream f(path, std::ios::app);
    if (!f.is_open()) {
        if (Logger::isInitialized())
            UNICALIB_WARN("无法写入精度 CSV: {}", path);
        return;
    }

    if (!file_existed)
        write_header(f, extra_keys);
    write_row(f, timestamp, success, residual_rms, elapsed_ms, extra, extra_keys);
}

void append_stage_result_accuracy(
    const std::string& output_dir,
    CalibAccuracyTask task,
    bool success,
    double residual_rms,
    double elapsed_ms,
    const std::string& message_short) {
    std::map<std::string, std::string> extra;
    if (!message_short.empty())
        extra["message"] = message_short.size() > 80 ? message_short.substr(0, 77) + "..." : message_short;
    append_calib_accuracy(output_dir, task, success, residual_rms, elapsed_ms, extra);
}

}  // namespace ns_unicalib
