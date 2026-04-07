/**
 * UniCalib C++ — DataManager 实现 (目录模式 + CSV IMU，含异常处理与日志)
 */
#include "unicalib/data_manager.hpp"
#include "unicalib/exceptions.hpp"
#include "unicalib/logger.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>

namespace unicalib {

namespace fs = std::filesystem;

DataManager::DataManager(const std::string& data_path) : data_path_(data_path) {
  is_bag_ = detect_bag();
}

bool DataManager::detect_bag() const {
  fs::path p(data_path_);
  if (!fs::exists(p))
    return false;
  if (fs::is_regular_file(p))
    return p.extension() == ".db3" || p.extension() == ".mcap";
  return fs::exists(p / "metadata.yaml");
}

void DataManager::open() {
  if (is_bag_) {
    LOG_WARNING("rosbag not supported in C++ build, using directory mode.");
    is_bag_ = false;
  }
  fs::path p(data_path_);
  if (!fs::exists(p))
    throw DataException(ErrorCode::DATA_DIRECTORY_NOT_FOUND, "Data path does not exist: " + data_path_);
  if (!fs::is_directory(p) && !fs::is_regular_file(p))
    throw DataException(ErrorCode::DATA_DIRECTORY_NOT_FOUND, "Data path is not a directory: " + data_path_);
}

void DataManager::close() {}

static std::string topic_to_dir(const std::string& topic) {
  std::string s = topic;
  while (!s.empty() && s[0] == '/') s.erase(0, 1);
  for (char& c : s) if (c == '/') c = '_';
  return s;
}

static std::string topic_tail(const std::string& topic) {
  auto pos = topic.find_last_of('/');
  if (pos == std::string::npos) return topic;
  return topic.substr(pos + 1);
}

void DataManager::iter_images(
  const std::string& topic,
  std::function<void(double, const ImageFrame&)> callback,
  std::optional<int> max_frames,
  int skip) {
  fs::path base(data_path_);
  fs::path topic_dir = base / topic_to_dir(topic);
  if (!fs::exists(topic_dir)) topic_dir = base / topic_tail(topic);
  if (!fs::exists(topic_dir)) return;

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(topic_dir)) {
    if (!e.is_regular_file()) continue;
    std::string ext = e.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp")
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());

  int emitted = 0;
  for (size_t i = 0; i < files.size(); ++i) {
    if (skip > 1 && (i % static_cast<size_t>(skip)) != 0) continue;
    // 占位：不依赖 OpenCV 时仅传递路径或空 data；调用方可自行读图
    ImageFrame frame;
    try {
      frame.timestamp = std::stod(files[i].stem().string());
    } catch (...) {
      frame.timestamp = static_cast<double>(i);
    }
    frame.width = 0;
    frame.height = 0;
    frame.data.clear();
    callback(frame.timestamp, frame);
    if (max_frames && ++emitted >= *max_frames) break;
  }
}

void DataManager::iter_image_paths(
  const std::string& topic,
  std::function<void(double, const std::string&)> callback,
  std::optional<int> max_frames,
  int skip) {
  fs::path base(data_path_);
  fs::path topic_dir = base / topic_to_dir(topic);
  if (!fs::exists(topic_dir)) topic_dir = base / topic_tail(topic);
  if (!fs::exists(topic_dir)) return;

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(topic_dir)) {
    if (!e.is_regular_file()) continue;
    std::string ext = e.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp")
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());

  int emitted = 0;
  for (size_t i = 0; i < files.size(); ++i) {
    if (skip > 1 && (i % static_cast<size_t>(skip)) != 0) continue;
    double ts;
    try {
      ts = std::stod(files[i].stem().string());
    } catch (...) {
      ts = static_cast<double>(i);
    }
    callback(ts, files[i].string());
    if (max_frames && ++emitted >= *max_frames) break;
  }
}

void DataManager::iter_pointclouds(
  const std::string& topic,
  std::function<void(double, const PointCloudFrame&)> callback,
  std::optional<int> max_frames,
  int skip) {
  fs::path base(data_path_);
  fs::path pc_dir = base / topic_to_dir(topic);
  if (!fs::exists(pc_dir)) pc_dir = base / topic_tail(topic);
  if (!fs::exists(pc_dir)) return;

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(pc_dir)) {
    if (!e.is_regular_file()) continue;
    std::string ext = e.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".pcd" || ext == ".bin" || ext == ".ply") files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());

  int emitted = 0;
  for (size_t i = 0; i < files.size(); ++i) {
    if (skip > 1 && (i % static_cast<size_t>(skip)) != 0) continue;
    PointCloudFrame frame;
    try {
      frame.timestamp = std::stod(files[i].stem().string());
    } catch (...) {
      frame.timestamp = static_cast<double>(i);
    }
    callback(frame.timestamp, frame);
    if (max_frames && ++emitted >= *max_frames) break;
  }
}

std::optional<IMUData> DataManager::load_imu_data(const std::string& topic) {
  fs::path base(data_path_);
  fs::path imu_csv = base / "imu.csv";
  if (!fs::exists(imu_csv)) imu_csv = base / (topic_tail(topic) + ".csv");
  if (!fs::exists(imu_csv)) {
    LOG_DEBUG("IMU CSV not found: " + imu_csv.string());
    return std::nullopt;
  }

  IMUData out;
  out.sample_rate = 200.0;
  std::ifstream f(imu_csv.string());
  if (!f.is_open())
    throw DataException(ErrorCode::DATA_FILE_NOT_FOUND, "Cannot open file: " + imu_csv.string());

  std::string line;
  if (!std::getline(f, line)) {
    LOG_WARNING("IMU CSV empty or no header: " + imu_csv.string());
    return std::nullopt;
  }
  size_t skipped = 0;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::string cell;
    double ts = 0, gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
    try {
      if (!std::getline(ss, cell, ',')) continue;
      ts = std::stod(cell);
      if (std::getline(ss, cell, ',')) gx = std::stod(cell);
      if (std::getline(ss, cell, ',')) gy = std::stod(cell);
      if (std::getline(ss, cell, ',')) gz = std::stod(cell);
      if (std::getline(ss, cell, ',')) ax = std::stod(cell);
      if (std::getline(ss, cell, ',')) ay = std::stod(cell);
      if (std::getline(ss, cell, ',')) az = std::stod(cell);
    } catch (const std::exception&) {
      ++skipped;
      continue;
    }
    out.timestamps.push_back(ts);
    out.gyro.push_back({gx, gy, gz});
    out.accel.push_back({ax, ay, az});
  }
  if (skipped > 0)
    LOG_WARNING("load_imu_data: skipped " + std::to_string(skipped) + " malformed lines in " + imu_csv.string());
  if (out.timestamps.empty()) {
    LOG_WARNING("IMU CSV has no valid data rows: " + imu_csv.string());
    return std::nullopt;
  }
  if (out.timestamps.size() > 1) {
    std::vector<double> diffs;
    for (size_t i = 1; i < out.timestamps.size(); ++i)
      diffs.push_back(out.timestamps[i] - out.timestamps[i - 1]);
    std::sort(diffs.begin(), diffs.end());
    size_t mid = diffs.size() / 2;
    double dt = diffs[mid];
    if (dt > 1e-9) out.sample_rate = 1.0 / dt;
  }
  return out;
}

}  // namespace unicalib
