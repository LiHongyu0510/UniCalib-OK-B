/**
 * UniCalib C++ — 数据管理接口 (rosbag/目录)
 * 对应 Python: unicalib/core/data_manager.py
 * 本实现先支持目录模式与 CSV IMU；rosbag 可通过插件或后续扩展。
 */
#pragma once

#include <string>
#include <vector>
#include <optional>
#include <functional>
#include "sensor_config.hpp"
#include "allan_variance.hpp"

namespace unicalib {

/** 单帧图像: (timestamp_sec, 图像数据). 图像以 row-major BGR 存储，width*height*3 */
struct ImageFrame {
  double timestamp = 0.0;
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;  // BGR
};

/** 点云帧: (timestamp_sec, N×3 或 N×4) */
struct PointCloudFrame {
  double timestamp = 0.0;
  std::vector<std::vector<double>> points;  // 每点 3 或 4 维
};

class DataManager {
 public:
  explicit DataManager(const std::string& data_path);
  ~DataManager() = default;

  void open();
  void close();

  /** 是否当前为 bag 数据源 (C++ 首版仅目录) */
  bool is_bag() const { return is_bag_; }

  /** 当前数据目录路径（用于验证/子进程等） */
  const std::string& data_path() const { return data_path_; }

  /** 迭代图像帧 (topic 对应子目录名或 topic 最后一段) */
  void iter_images(
    const std::string& topic,
    std::function<void(double, const ImageFrame&)> callback,
    std::optional<int> max_frames = std::nullopt,
    int skip = 1);

  /** 迭代图像文件路径 (用于需自行读图的标定器，如鱼眼 OpenCV) */
  void iter_image_paths(
    const std::string& topic,
    std::function<void(double, const std::string&)> callback,
    std::optional<int> max_frames = std::nullopt,
    int skip = 1);

  /** 迭代点云帧 */
  void iter_pointclouds(
    const std::string& topic,
    std::function<void(double, const PointCloudFrame&)> callback,
    std::optional<int> max_frames = std::nullopt,
    int skip = 1);

  /** 加载 IMU 数据 (目录模式: imu.csv 或 topic 名.csv) */
  std::optional<IMUData> load_imu_data(const std::string& topic);

 private:
  std::string data_path_;
  bool is_bag_ = false;

  bool detect_bag() const;
};

}  // namespace unicalib
