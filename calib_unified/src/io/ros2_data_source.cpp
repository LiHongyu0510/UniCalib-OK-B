/**
 * UniCalib Unified — ROS2 数据源实现
 * 
 * 实现两种模式:
 *   1. ROS2 bag 文件读取 (使用 rosbag2_cpp)
 *   2. ROS2 实时话题订阅 (使用 rclcpp)
 */

#include "unicalib/io/ros2_data_source.h"
#include "unicalib/common/logger.h"
#include "unicalib/io/yaml_io.h"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <set>

#ifdef UNICALIB_WITH_ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#endif

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===========================================================================
// Ros2BagDataSource 实现
// ===========================================================================

Ros2BagDataSource::Ros2BagDataSource(const RosDataSourceConfig& cfg)
    : RosDataSourceBase(cfg) {
    status_msg_ = "未初始化";
}

Ros2BagDataSource::~Ros2BagDataSource() {
    bag_reader_.reset();
}

double Ros2BagDataSource::stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool Ros2BagDataSource::read_metadata_yaml(const std::string& bag_file) {
    // ROS2 rosbag2 格式：bag 为目录，metadata.yaml 在目录内
    fs::path bag_path(bag_file);
    fs::path metadata_path;
    if (fs::is_directory(bag_path)) {
        metadata_path = bag_path / "metadata.yaml";
    } else {
        metadata_path = bag_path.parent_path() / "metadata.yaml";
    }
    
    if (!fs::exists(metadata_path)) {
        UNICALIB_INFO("[Ros2BagDataSource] 未找到 metadata.yaml: {}", metadata_path.string());
        return false;
    }
    
    UNICALIB_INFO("[Ros2BagDataSource] 正在读取 metadata.yaml: {}", metadata_path.string());
    
    try {
        YAML::Node root = YAML::LoadFile(metadata_path.string());
        
        // 查找 topics_with_message_count：兼容根节点 或 rosbag2_bagfile_information 下（标准 rosbag2 格式）
        YAML::Node topics_node = root["topics_with_message_count"];
        if (!topics_node) {
            if (root["rosbag2_bagfile_information"]) {
                topics_node = root["rosbag2_bagfile_information"]["topics_with_message_count"];
            }
        }
        if (!topics_node) {
            UNICALIB_WARN("[Ros2BagDataSource] metadata.yaml 中缺少 topics_with_message_count 节");
            return false;
        }
        
        // 收集所有话题及其消息数量与类型（兼容两种 metadata 格式：name 在顶层 或 在 topic_metadata 下）
        std::map<std::string, size_t> topic_msg_counts;
        bag_topic_msg_counts_.clear();
        bag_topic_types_.clear();
        for (const auto& topic_node : topics_node) {
            std::string topic_name;
            if (topic_node["name"])
                topic_name = topic_node["name"].as<std::string>();
            else if (topic_node["topic_metadata"] && topic_node["topic_metadata"]["name"])
                topic_name = topic_node["topic_metadata"]["name"].as<std::string>();
            else
                continue;
            size_t msg_count = topic_node["message_count"] ? topic_node["message_count"].as<size_t>() : 0;
            topic_msg_counts[topic_name] = msg_count;
            bag_topic_msg_counts_[topic_name] = msg_count;
            std::string topic_type;
            if (topic_node["topic_metadata"] && topic_node["topic_metadata"]["type"])
                topic_type = topic_node["topic_metadata"]["type"].as<std::string>();
            else if (topic_node["type"])
                topic_type = topic_node["type"].as<std::string>();
            if (!topic_type.empty())
                bag_topic_types_[topic_name] = topic_type;
            UNICALIB_DEBUG("[Ros2BagDataSource] 找到话题: {} (类型: {}, 消息数: {})",
                         topic_name, topic_type.empty() ? "?" : topic_type, msg_count);
        }
        
        // 自动检测话题类型
        auto_detect_topics(topic_msg_counts);
        
        return true;
        
    } catch (const std::exception& e) {
        UNICALIB_ERROR("[Ros2BagDataSource] 读取 metadata.yaml 失败: {}", e.what());
        return false;
    }
}

void Ros2BagDataSource::auto_detect_topics(
    const std::map<std::string, size_t>& topic_msg_counts) {

    // 1) 优先使用配置中的单话题（来自配置文件 sensors[].topic 或 ros2.lidar_topic 等）
    if (!cfg_.lidar_ros2_topic.empty()) {
        topic_mapping_.lidar_topic = cfg_.lidar_ros2_topic;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 LiDAR 话题: {}", cfg_.lidar_ros2_topic);
    } else if (!cfg_.lidar_topics.empty()) {
        topic_mapping_.lidar_topic = cfg_.lidar_topics.begin()->second;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 LiDAR 话题 (来自 sensors): {}", topic_mapping_.lidar_topic);
    } else {
        std::string detected_lidar = find_best_lidar_topic(topic_msg_counts);
        if (!detected_lidar.empty()) {
            topic_mapping_.lidar_topic = detected_lidar;
            UNICALIB_INFO("[Ros2BagDataSource] 自动检测 LiDAR 话题: {}", detected_lidar);
        }
    }
    
    if (!cfg_.camera_ros2_topic.empty()) {
        topic_mapping_.camera_topic = cfg_.camera_ros2_topic;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的相机话题: {}", cfg_.camera_ros2_topic);
    } else if (!cfg_.camera_topics.empty()) {
        topic_mapping_.camera_topic = cfg_.camera_topics.begin()->second;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的相机话题 (来自 sensors): {}", topic_mapping_.camera_topic);
    } else {
        std::string detected_camera = find_best_camera_topic(topic_msg_counts);
        if (!detected_camera.empty()) {
            topic_mapping_.camera_topic = detected_camera;
            UNICALIB_INFO("[Ros2BagDataSource] 自动检测相机话题: {}", detected_camera);
        }
    }
    
    if (!cfg_.imu_ros2_topic.empty()) {
        topic_mapping_.imu_topic = cfg_.imu_ros2_topic;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 IMU 话题: {}", cfg_.imu_ros2_topic);
    } else if (!cfg_.imu_topics.empty()) {
        topic_mapping_.imu_topic = cfg_.imu_topics.begin()->second;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 IMU 话题 (来自 sensors): {}", topic_mapping_.imu_topic);
    } else {
        std::string detected_imu = find_best_imu_topic(topic_msg_counts);
        if (!detected_imu.empty()) {
            topic_mapping_.imu_topic = detected_imu;
            UNICALIB_INFO("[Ros2BagDataSource] 自动检测 IMU 话题: {}", detected_imu);
        }
    }
}

std::string Ros2BagDataSource::find_best_lidar_topic(
    const std::map<std::string, size_t>& topic_msg_counts) {

    // LiDAR 话题关键词
    std::vector<std::string> lidar_keywords = {
        "pointcloud", "lidar", "velodyne", "ouster", "livox", "pandar",
        "points", "scan", "os_cloud"
    };
    
    // 优先级关键词（更具体）
    std::vector<std::string> priority_keywords = {
        "velodyne_points", "os_cloud_node1/points", "os_cloud_node2/points",
        "ouster/points", "livox/lidar"
    };
    
    // 查找匹配的话题
    for (const auto& [topic, count] : topic_msg_counts) {
        std::string topic_lower = topic;
        std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(), ::tolower);
        
        // 首先检查优先级关键词（更准确）
        for (const auto& keyword : priority_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配高优先级 LiDAR 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
        
        // 然后检查通用 LiDAR 关键词
        for (const auto& keyword : lidar_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配 LiDAR 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
    }
    
    // 如果没有找到精确匹配，返回空
    return "";
}

std::string Ros2BagDataSource::find_best_camera_topic(
    const std::map<std::string, size_t>& topic_msg_counts) {
    
    // 相机话题关键词
    std::vector<std::string> camera_keywords = {
        "image", "camera", "left", "right", "front", "rear",
        "rgb", "ir", "mono", "stereo", "color"
    };
    
    // 优先级关键词（更具体）
    std::vector<std::string> priority_keywords = {
        "left/image_raw", "right/image_raw",
        "/cam_left/image_raw", "/cam_right/image_raw",
        "left/image_raw/compressed", "right/image_raw/compressed"
    };
    
    // 查找匹配的话题
    for (const auto& [topic, count] : topic_msg_counts) {
        std::string topic_lower = topic;
        std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(), ::tolower);
        
        // 首先检查优先级关键词（更准确）
        for (const auto& keyword : priority_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                // 优先选择包含 "left" 的相机话题（假设为左相机）
                if (topic_lower.find("left") != std::string::npos) {
                    UNICALIB_DEBUG("[Ros2BagDataSource] 匹配高优先级左相机话题: {} (关键词: {})", 
                                 topic, keyword);
                    return topic;
                }
            }
        }
        
        // 然后检查通用相机关键词
        for (const auto& keyword : camera_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配相机话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
    }
    
    return "";
}

std::string Ros2BagDataSource::find_best_imu_topic(
    const std::map<std::string, size_t>& topic_msg_counts) {
    
    // IMU 话题关键词
    std::vector<std::string> imu_keywords = {
        "imu", "gyro", "accel", "magnetic", "angular", "linear",
        "attitude", "orientation", "rotation"
    };
    
    // 优先级关键词（常见 IMU 话题名称）
    std::vector<std::string> priority_keywords = {
        "/imu/imu", "/dji_sdk/imu", "/imu/imu",
        "/imu/magnetic_field", "/imu/temperature"
    };
    
    // 查找匹配的话题
    for (const auto& [topic, count] : topic_msg_counts) {
        std::string topic_lower = topic;
        std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(), ::tolower);
        
        // 首先检查优先级关键词（更准确）
        for (const auto& keyword : priority_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配高优先级 IMU 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
        
        // 然后检查通用 IMU 关键词
        for (const auto& keyword : imu_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配 IMU 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
    }
    
    return "";
}

LiDARScanRos Ros2BagDataSource::parse_point_cloud2(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2>& msg) {
    
    LiDARScanRos scan;
    scan.timestamp = stamp_to_sec(msg->header.stamp);
    
    // 转换为 PCL 点云
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    // 过滤点
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
        // 过滤 NaN
        if (cfg_.filter_nan && (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))) {
            continue;
        }
        
        // 距离过滤
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (dist < cfg_.min_range || dist > cfg_.max_range) {
            continue;
        }
        
        filtered.push_back(pt);
    }
    
    scan.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>(filtered));
    
    return scan;
}

CameraFrameRos Ros2BagDataSource::parse_image(
    const std::shared_ptr<sensor_msgs::msg::Image>& msg) {
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        return CameraFrameRos(stamp_to_sec(msg->header.stamp), cv_ptr->image);
    } catch (const cv_bridge::Exception& e) {
        UNICALIB_WARN("图像转换失败: {}", e.what());
        return CameraFrameRos(stamp_to_sec(msg->header.stamp), cv::Mat());
    }
}

IMUFrameRos Ros2BagDataSource::parse_imu(
    const std::shared_ptr<sensor_msgs::msg::Imu>& msg) {
    
    IMUFrameRos frame;
    frame.timestamp = stamp_to_sec(msg->header.stamp);
    
    auto safe_gyro = [&msg](int i) -> double {
        double v = (i == 0) ? msg->angular_velocity.x : (i == 1) ? msg->angular_velocity.y : msg->angular_velocity.z;
        return (std::isnan(v) || std::isinf(v)) ? 0.0 : v;
    };
    auto safe_accel = [&msg](int i) -> double {
        double v = (i == 0) ? msg->linear_acceleration.x : (i == 1) ? msg->linear_acceleration.y : msg->linear_acceleration.z;
        return (std::isnan(v) || std::isinf(v)) ? 0.0 : v;
    };
    for (int i = 0; i < 3; ++i) {
        frame.gyro[i] = safe_gyro(i);
        frame.accel[i] = safe_accel(i);
    }
    return frame;
}

bool Ros2BagDataSource::load() {
    status_msg_ = "正在加载 bag...";
    
    if (cfg_.bag_file.empty()) {
        status_msg_ = "未指定 bag 文件";
        return false;
    }
    
    if (!fs::exists(cfg_.bag_file)) {
        status_msg_ = "bag 文件不存在: " + cfg_.bag_file;
        return false;
    }
    
    UNICALIB_INFO("[Ros2BagDataSource] 打开 bag 文件: {}", cfg_.bag_file);
    
    // ─── 步骤 1: 从 metadata.yaml 自动检测话题 ─────────
    if (!read_metadata_yaml(cfg_.bag_file)) {
        status_msg_ = "读取 metadata.yaml 失败";
        return false;
    }
    
    // ─── 步骤 2: 打开 bag 文件 ────────────────────────────────────────
    try {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = cfg_.bag_file;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        bag_reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
        bag_reader_->open(storage_options, converter_options);

        UNICALIB_INFO("[Ros2BagDataSource] Bag 已打开，开始读取消息...");

        // 获取所有话题信息 (TopicMetadata 有 name, type 等字段)
        const auto& topics = bag_reader_->get_all_topics_and_types();
        UNICALIB_INFO("[Ros2BagDataSource] 找到 {} 个话题", topics.size());
        for (const auto& meta : topics) {
            auto it_count = bag_topic_msg_counts_.find(meta.name);
            size_t count = (it_count != bag_topic_msg_counts_.end()) ? it_count->second : 0;
            UNICALIB_INFO("[Ros2BagDataSource]  话题: {}  类型: {}  消息数: {}",
                         meta.name, meta.type, count);
        }

        // 按类型收集 bag 内所有话题，便于“接收所有、选择使用”
        const std::string imu_type = "sensor_msgs/msg/Imu";
        const std::string pc2_type = "sensor_msgs/msg/PointCloud2";
        const std::string img_type = "sensor_msgs/msg/Image";
        std::map<std::string, std::string> topic_to_type;
        for (const auto& meta : topics) {
            topic_to_type[meta.name] = meta.type;
        }

        UNICALIB_INFO("[Ros2BagDataSource] 将加载 bag 内所有 IMU / LiDAR / 相机话题，加载后可通过 sensor_id 或话题名选择使用");

        // 反序列化器 (ROS2 Humble 需手动反序列化 SerializedBagMessage)
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_pc;
        rclcpp::Serialization<sensor_msgs::msg::Image> ser_img;
        rclcpp::Serialization<sensor_msgs::msg::Imu> ser_imu;

        bool time_window_set = (cfg_.time_window_end > cfg_.time_window_start);
        double last_time = 0.0;

        while (bag_reader_->has_next()) {
            auto msg = bag_reader_->read_next();
            double msg_time = static_cast<double>(msg->time_stamp) * 1e-9;

            if (time_window_set) {
                if (msg_time < cfg_.time_window_start) continue;
                if (msg_time > cfg_.time_window_end) break;
            }

            last_time = std::max(last_time, msg_time);
            std::string topic = msg->topic_name;
            auto it_type = topic_to_type.find(topic);
            if (it_type == topic_to_type.end() || !msg->serialized_data) continue;

            const std::string& type = it_type->second;

            // LiDAR：所有 PointCloud2 话题
            if (type == pc2_type) {
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                ser_pc.deserialize_message(&serialized_msg, cloud_msg.get());
                lidar_data_[topic].push_back(parse_point_cloud2(cloud_msg));
                // 仅每 500 帧打印一次，避免日志膨胀
                if (lidar_data_[topic].size() % 500 == 0) {
                    UNICALIB_DEBUG("  收集 LiDAR [{}] 帧数: {}", topic, lidar_data_[topic].size());
                }
                continue;
            }

            // 相机：所有 Image 话题
            if (type == img_type) {
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
                ser_img.deserialize_message(&serialized_msg, image_msg.get());
                camera_data_[topic].push_back(parse_image(image_msg));
                // 仅每 500 帧打印一次，避免日志膨胀
                if (camera_data_[topic].size() % 500 == 0) {
                    UNICALIB_DEBUG("  收集相机 [{}] 帧数: {}", topic, camera_data_[topic].size());
                }
                continue;
            }

            // IMU：所有 Imu 话题
            if (type == imu_type) {
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
                ser_imu.deserialize_message(&serialized_msg, imu_msg.get());
                imu_data_[topic].push_back(parse_imu(imu_msg));
                if (imu_msg->header.stamp.sec == 0 && imu_msg->header.stamp.nanosec == 0) {
                    imu_data_[topic].back().timestamp = msg_time;
                }
                // 仅每 5000 帧打印一次（IMU 帧率高），避免日志膨胀
                if (imu_data_[topic].size() % 5000 == 0) {
                    UNICALIB_DEBUG("  收集 IMU [{}] 帧数: {}", topic, imu_data_[topic].size());
                }
            }
        }

        // 构建“选择使用”映射：sensor_id -> 实际话题名（配置优先，否则首个有数据的话题）
        auto pick_first = [](const std::map<std::string, std::vector<LiDARScanRos>>& data) -> std::string {
            if (data.empty()) return {};
            for (const auto& [k, v] : data) { if (!v.empty()) return k; }
            return {};
        };
        auto pick_first_cam = [](const std::map<std::string, std::vector<CameraFrameRos>>& data) -> std::string {
            if (data.empty()) return {};
            for (const auto& [k, v] : data) { if (!v.empty()) return k; }
            return {};
        };
        auto pick_first_imu = [](const std::map<std::string, std::vector<IMUFrameRos>>& data) -> std::string {
            if (data.empty()) return {};
            for (const auto& [k, v] : data) { if (!v.empty()) return k; }
            return {};
        };

        effective_lidar_topic_.clear();
        std::vector<std::string> missing_lidar;  // (sensor_id: topic) 用于 strict 模式报错
        for (const auto& [sensor_id, topic] : cfg_.lidar_topics) {
            if (!topic.empty() && lidar_data_.count(topic) && !lidar_data_.at(topic).empty()) {
                effective_lidar_topic_[sensor_id] = topic;
            } else {
                if (cfg_.strict_topic_match && !topic.empty()) {
                    missing_lidar.push_back(sensor_id + ": " + topic);
                } else {
                    std::string fallback = pick_first(lidar_data_);
                    if (!fallback.empty()) {
                        effective_lidar_topic_[sensor_id] = fallback;
                        if (!topic.empty() && topic != fallback) {
                            UNICALIB_WARN("[Ros2BagDataSource] LiDAR '{}' 配置话题 '{}' 无数据，已选用: {}", sensor_id, topic, fallback);
                        }
                    }
                }
            }
        }
        if (cfg_.lidar_topics.empty() && !lidar_data_.empty()) {
            effective_lidar_topic_["lidar_0"] = pick_first(lidar_data_);
        }

        effective_camera_topic_.clear();
        std::vector<std::string> missing_camera;
        for (const auto& [sensor_id, topic] : cfg_.camera_topics) {
            if (!topic.empty() && camera_data_.count(topic) && !camera_data_.at(topic).empty()) {
                effective_camera_topic_[sensor_id] = topic;
            } else {
                if (cfg_.strict_topic_match && !topic.empty()) {
                    missing_camera.push_back(sensor_id + ": " + topic);
                } else {
                    std::string fallback = pick_first_cam(camera_data_);
                    if (!fallback.empty()) {
                        effective_camera_topic_[sensor_id] = fallback;
                        if (!topic.empty() && topic != fallback) {
                            UNICALIB_WARN("[Ros2BagDataSource] 相机 '{}' 配置话题 '{}' 无数据，已选用: {}", sensor_id, topic, fallback);
                        }
                    }
                }
            }
        }
        if (cfg_.camera_topics.empty() && !camera_data_.empty()) {
            effective_camera_topic_["camera_0"] = pick_first_cam(camera_data_);
        }

        effective_imu_topic_.clear();
        std::vector<std::string> missing_imu;
        for (const auto& [sensor_id, topic] : cfg_.imu_topics) {
            if (!topic.empty() && imu_data_.count(topic) && !imu_data_.at(topic).empty()) {
                effective_imu_topic_[sensor_id] = topic;
            } else {
                if (cfg_.strict_topic_match && !topic.empty()) {
                    missing_imu.push_back(sensor_id + ": " + topic);
                } else {
                    std::string fallback = pick_first_imu(imu_data_);
                    if (!fallback.empty()) {
                        effective_imu_topic_[sensor_id] = fallback;
                        if (!topic.empty() && topic != fallback) {
                            UNICALIB_WARN("[Ros2BagDataSource] IMU '{}' 配置话题 '{}' 无数据，已选用: {}", sensor_id, topic, fallback);
                        }
                    }
                }
            }
        }
        if (cfg_.imu_topics.empty() && !imu_data_.empty()) {
            effective_imu_topic_["imu_0"] = pick_first_imu(imu_data_);
        }

        if (cfg_.strict_topic_match && (!missing_lidar.empty() || !missing_camera.empty() || !missing_imu.empty())) {
            std::string msg = "[Ros2BagDataSource] 以配置文件为准：以下配置话题在 bag 中无数据，请使配置与 bag 一致（或设置 ros2.strict_topic_match: false 允许自动回退）。";
            if (!missing_lidar.empty()) {
                for (const auto& s : missing_lidar) msg += " LiDAR[" + s + "]";
            }
            if (!missing_camera.empty()) {
                for (const auto& s : missing_camera) msg += " 相机[" + s + "]";
            }
            if (!missing_imu.empty()) {
                for (const auto& s : missing_imu) msg += " IMU[" + s + "]";
            }
            status_msg_ = msg;
            UNICALIB_ERROR("{}", status_msg_);
            return false;
        }
        
        // 记录时间范围：从所有已加载话题取最小/最大时间戳
        start_time_ = 0.0;
        end_time_ = 0.0;
        auto update_range = [](double& t_start, double& t_end,
                               const std::map<std::string, std::vector<LiDARScanRos>>& data) {
            for (const auto& [_, vec] : data) {
                if (vec.empty()) continue;
                if (t_start == 0.0 && t_end == 0.0) {
                    t_start = vec.front().timestamp;
                    t_end = vec.back().timestamp;
                } else {
                    t_start = std::min(t_start, vec.front().timestamp);
                    t_end = std::max(t_end, vec.back().timestamp);
                }
            }
        };
        update_range(start_time_, end_time_, lidar_data_);
        for (const auto& [_, frames] : camera_data_) {
            if (frames.empty()) continue;
            if (start_time_ == 0.0 && end_time_ == 0.0) {
                start_time_ = frames.front().timestamp;
                end_time_ = frames.back().timestamp;
            } else {
                start_time_ = std::min(start_time_, frames.front().timestamp);
                end_time_ = std::max(end_time_, frames.back().timestamp);
            }
        }
        for (const auto& [_, frames] : imu_data_) {
            if (frames.empty()) continue;
            if (start_time_ == 0.0 && end_time_ == 0.0) {
                start_time_ = frames.front().timestamp;
                end_time_ = frames.back().timestamp;
            } else {
                start_time_ = std::min(start_time_, frames.front().timestamp);
                end_time_ = std::max(end_time_, frames.back().timestamp);
            }
        }
        
        UNICALIB_INFO("[Ros2BagDataSource] 加载完成 (所有话题按话题名存储，可通过 sensor_id 或话题名选择):");
        for (const auto& [topic, data] : lidar_data_) {
            UNICALIB_INFO("  LiDAR [{}]: {} 帧", topic, data.size());
        }
        for (const auto& [topic, data] : camera_data_) {
            UNICALIB_INFO("  相机 [{}]: {} 帧", topic, data.size());
        }
        for (const auto& [topic, data] : imu_data_) {
            UNICALIB_INFO("  IMU [{}]: {} 帧", topic, data.size());
        }
        for (const auto& [sensor_id, topic] : effective_lidar_topic_) {
            UNICALIB_INFO("  选用 LiDAR  {} -> {}", sensor_id, topic);
        }
        for (const auto& [sensor_id, topic] : effective_camera_topic_) {
            UNICALIB_INFO("  选用 相机  {} -> {}", sensor_id, topic);
        }
        for (const auto& [sensor_id, topic] : effective_imu_topic_) {
            UNICALIB_INFO("  选用 IMU    {} -> {}", sensor_id, topic);
        }
        UNICALIB_INFO("  时间范围: {:.3f} - {:.3f} 秒", start_time_, end_time_);
        
        // ─── 时间对齐：截取 IMU/LiDAR 重叠区间 ────────────────────────────────────────
        // 当首/末帧差异较大时，强制截取重叠区间，避免时间基准不一致导致标定偏差
        if (!lidar_data_.empty() && !imu_data_.empty()) {
            // 计算各传感器的时间范围
            double imu_t_min = 1e9, imu_t_max = 0;
            double lidar_t_min = 1e9, lidar_t_max = 0;
            for (const auto& [_, frames] : imu_data_) {
                if (!frames.empty()) {
                    imu_t_min = std::min(imu_t_min, frames.front().timestamp);
                    imu_t_max = std::max(imu_t_max, frames.back().timestamp);
                }
            }
            for (const auto& [_, frames] : lidar_data_) {
                if (!frames.empty()) {
                    lidar_t_min = std::min(lidar_t_min, frames.front().timestamp);
                    lidar_t_max = std::max(lidar_t_max, frames.back().timestamp);
                }
            }
            
            double imu_lidar_begin_diff = std::abs(imu_t_min - lidar_t_min);
            double imu_lidar_end_diff = std::abs(imu_t_max - lidar_t_max);
            
            // 如果首末帧差异超过阈值（如 1s），截取重叠区间
            if (imu_lidar_begin_diff > 1.0 || imu_lidar_end_diff > 1.0) {
                double overlap_begin = std::max(imu_t_min, lidar_t_min);
                double overlap_end = std::min(imu_t_max, lidar_t_max);
                
                if (overlap_end > overlap_begin) {
                    UNICALIB_INFO("[Time-Align] 检测到首/末帧差异较大 (首 {:.3f}s 末 {:.3f}s)，截取重叠区间",
                                  imu_lidar_begin_diff, imu_lidar_end_diff);
                    UNICALIB_INFO("[Time-Align] 原 IMU: [{:.3f}, {:.3f}] LiDAR: [{:.3f}, {:.3f}]",
                                  imu_t_min, imu_t_max, lidar_t_min, lidar_t_max);
                    UNICALIB_INFO("[Time-Align] 截取区间: [{:.3f}, {:.3f}] 跨度 {:.1f}s",
                                  overlap_begin, overlap_end, overlap_end - overlap_begin);
                    
                    // 截取 IMU 数据
                    for (auto& [topic, frames] : imu_data_) {
                        std::vector<IMUFrameRos> filtered;
                        filtered.reserve(frames.size());
                        for (const auto& f : frames) {
                            if (f.timestamp >= overlap_begin && f.timestamp <= overlap_end) {
                                filtered.push_back(f);
                            }
                        }
                        if (filtered.size() != frames.size()) {
                            UNICALIB_INFO("[Time-Align] IMU [{}] 截取: {} -> {} 帧",
                                          topic, frames.size(), filtered.size());
                            frames = std::move(filtered);
                        }
                    }
                    
                    // 截取 LiDAR 数据
                    for (auto& [topic, frames] : lidar_data_) {
                        std::vector<LiDARScanRos> filtered;
                        filtered.reserve(frames.size());
                        for (const auto& f : frames) {
                            if (f.timestamp >= overlap_begin && f.timestamp <= overlap_end) {
                                filtered.push_back(f);
                            }
                        }
                        if (filtered.size() != frames.size()) {
                            UNICALIB_INFO("[Time-Align] LiDAR [{}] 截取: {} -> {} 帧",
                                          topic, frames.size(), filtered.size());
                            frames = std::move(filtered);
                        }
                    }
                    
                    // 更新时间范围
                    start_time_ = overlap_begin;
                    end_time_ = overlap_end;

                    // 截断后汇总：便于日志确认标定使用的数据范围
                    size_t imu_total = 0, lidar_total = 0;
                    for (const auto& [_, frames] : imu_data_) imu_total += frames.size();
                    for (const auto& [_, frames] : lidar_data_) lidar_total += frames.size();
                    UNICALIB_INFO("[Time-Align] 截断后: 时间范围 [{:.3f}, {:.3f}] 跨度 {:.1f}s | IMU 总帧数 {} | LiDAR 总帧数 {}",
                                  start_time_, end_time_, end_time_ - start_time_, imu_total, lidar_total);
                }
            }
        }
        
        // 采样和帧数限制
        if (cfg_.max_frames > 0 || cfg_.sample_interval > 0.0) {
            UNICALIB_INFO("[Ros2BagDataSource] 应用采样/帧数限制...");
            for (auto& [sensor_id, data] : lidar_data_) {
                if (cfg_.max_frames > 0 && data.size() > cfg_.max_frames) {
                    // 均匀采样
                    size_t step = data.size() / cfg_.max_frames;
                    std::vector<LiDARScanRos> sampled;
                    for (size_t i = 0; i < data.size() && sampled.size() < cfg_.max_frames; i += step) {
                        sampled.push_back(data[i]);
                    }
                    data = std::move(sampled);
                    UNICALIB_INFO("  LiDAR {} 采样后: {} 帧", sensor_id, data.size());
                }
            }
            for (auto& [sensor_id, data] : camera_data_) {
                if (cfg_.max_frames > 0 && data.size() > cfg_.max_frames) {
                    size_t step = data.size() / cfg_.max_frames;
                    std::vector<CameraFrameRos> sampled;
                    for (size_t i = 0; i < data.size() && sampled.size() < cfg_.max_frames; i += step) {
                        sampled.push_back(data[i]);
                    }
                    data = std::move(sampled);
                    UNICALIB_INFO("  相机 {} 采样后: {} 帧", sensor_id, data.size());
                }
            }
        }
        
        loaded_ = true;
        status_msg_ = "加载成功";
        return true;
        
    } catch (const std::exception& e) {
        status_msg_ = "加载失败: " + std::string(e.what());
        UNICALIB_ERROR("[Ros2BagDataSource] {}", status_msg_);
        return false;
    }
}

std::vector<LiDARScanRos> Ros2BagDataSource::get_lidar_scans(
    const std::string& sensor_id) const {
    // 支持按话题名或配置的 sensor_id 选择
    auto it = lidar_data_.find(sensor_id);
    if (it != lidar_data_.end()) return it->second;
    auto it_eff = effective_lidar_topic_.find(sensor_id);
    if (it_eff != effective_lidar_topic_.end()) {
        it = lidar_data_.find(it_eff->second);
        if (it != lidar_data_.end()) return it->second;
    }
    return std::vector<LiDARScanRos>();
}

std::vector<CameraFrameRos> Ros2BagDataSource::get_camera_frames(
    const std::string& sensor_id) const {
    auto it = camera_data_.find(sensor_id);
    if (it != camera_data_.end()) return it->second;
    auto it_eff = effective_camera_topic_.find(sensor_id);
    if (it_eff != effective_camera_topic_.end()) {
        it = camera_data_.find(it_eff->second);
        if (it != camera_data_.end()) return it->second;
    }
    return std::vector<CameraFrameRos>();
}

std::vector<IMUFrameRos> Ros2BagDataSource::get_imu_frames(
    const std::string& sensor_id) const {
    auto it = imu_data_.find(sensor_id);
    if (it != imu_data_.end()) return it->second;
    auto it_eff = effective_imu_topic_.find(sensor_id);
    if (it_eff != effective_imu_topic_.end()) {
        it = imu_data_.find(it_eff->second);
        if (it != imu_data_.end()) return it->second;
    }
    return std::vector<IMUFrameRos>();
}

std::vector<std::string> Ros2BagDataSource::get_lidar_ids() const {
    std::set<std::string> ids;
    for (const auto& [s, _] : effective_lidar_topic_) ids.insert(s);
    for (const auto& [topic, _] : lidar_data_) ids.insert(topic);
    return std::vector<std::string>(ids.begin(), ids.end());
}

std::vector<std::string> Ros2BagDataSource::get_camera_ids() const {
    std::set<std::string> ids;
    for (const auto& [s, _] : effective_camera_topic_) ids.insert(s);
    for (const auto& [topic, _] : camera_data_) ids.insert(topic);
    return std::vector<std::string>(ids.begin(), ids.end());
}

std::vector<std::string> Ros2BagDataSource::get_imu_ids() const {
    std::set<std::string> ids;
    for (const auto& [s, _] : effective_imu_topic_) ids.insert(s);
    for (const auto& [topic, _] : imu_data_) ids.insert(topic);
    return std::vector<std::string>(ids.begin(), ids.end());
}

// ===========================================================================
// Ros2RealtimeDataSource 实现
// ===========================================================================

Ros2RealtimeDataSource::Ros2RealtimeDataSource(const RosDataSourceConfig& cfg)
    : RosDataSourceBase(cfg) {
    status_msg_ = "未启动";
}

Ros2RealtimeDataSource::~Ros2RealtimeDataSource() {
    stop();
}

bool Ros2RealtimeDataSource::start() {
    if (running_) {
        UNICALIB_WARN("[Ros2RealtimeDataSource] 已在运行中");
        return true;
    }
    
    UNICALIB_INFO("[Ros2RealtimeDataSource] 启动 ROS2 节点...");
    
    try {
        // 创建 ROS2 节点
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("unicalib_data_loader");
        
        // 创建订阅者
        for (const auto& [sensor_id, topic] : cfg_.lidar_topics) {
            auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, 10,
                [this, sensor_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->lidar_callback(sensor_id, msg);
                });
            subscriptions_[sensor_id] = sub;
            UNICALIB_INFO("  订阅 LiDAR {}: {}", sensor_id, topic);
        }
        
        for (const auto& [sensor_id, topic] : cfg_.camera_topics) {
            auto sub = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, 10,
                [this, sensor_id](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->camera_callback(sensor_id, msg);
                });
            subscriptions_[sensor_id] = sub;
            UNICALIB_INFO("  订阅相机 {}: {}", sensor_id, topic);
        }
        
        for (const auto& [sensor_id, topic] : cfg_.imu_topics) {
            auto sub = node_->create_subscription<sensor_msgs::msg::Imu>(
                topic, 100,
                [this, sensor_id](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    this->imu_callback(sensor_id, msg);
                });
            subscriptions_[sensor_id] = sub;
            UNICALIB_INFO("  订阅 IMU {}: {}", sensor_id, topic);
        }
        
        // 创建执行器
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        
        // 在后台线程中运行
        stop_requested_ = false;
        std::thread exec_thread([this]() {
            while (rclcpp::ok() && !stop_requested_) {
                executor_->spin_once(std::chrono::milliseconds(10));
            }
        });
        exec_thread.detach();
        
        running_ = true;
        status_msg_ = "运行中";
        UNICALIB_INFO("[Ros2RealtimeDataSource] 已启动，等待数据...");
        return true;
        
    } catch (const std::exception& e) {
        status_msg_ = "启动失败: " + std::string(e.what());
        UNICALIB_ERROR("[Ros2RealtimeDataSource] {}", status_msg_);
        return false;
    }
}

void Ros2RealtimeDataSource::stop() {
    if (!running_) return;
    
    UNICALIB_INFO("[Ros2RealtimeDataSource] 停止...");
    stop_requested_ = true;
    
    // 清理订阅
    subscriptions_.clear();
    
    // 清理执行器和节点
    executor_.reset();
    node_.reset();
    
    // 清理 ROS2
    rclcpp::shutdown();
    
    running_ = false;
    status_msg_ = "已停止";
}

bool Ros2RealtimeDataSource::load() {
    return start();
}

bool Ros2RealtimeDataSource::wait_for_data(double timeout_sec) {
    UNICALIB_INFO("[Ros2RealtimeDataSource] 等待数据就绪 (超时: {:.1f} 秒)...", 
                  timeout_sec);
    
    auto start = std::chrono::steady_clock::now();
    
    while (running_ && !stop_requested_) {
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            
            // 检查是否已收集足够数据
            bool has_lidar = !cfg_.lidar_topics.empty();
            bool has_camera = !cfg_.camera_topics.empty();
            bool has_imu = !cfg_.imu_topics.empty();
            
            for (const auto& [sensor_id, _] : cfg_.lidar_topics) {
                if (!lidar_data_[sensor_id].empty()) has_lidar = true;
            }
            for (const auto& [sensor_id, _] : cfg_.camera_topics) {
                if (!camera_data_[sensor_id].empty()) has_camera = true;
            }
            for (const auto& [sensor_id, _] : cfg_.imu_topics) {
                if (!imu_data_[sensor_id].empty()) has_imu = true;
            }
            
            if ((has_lidar || cfg_.lidar_topics.empty()) &&
                (has_camera || cfg_.camera_topics.empty()) &&
                (has_imu || cfg_.imu_topics.empty())) {
                UNICALIB_INFO("[Ros2RealtimeDataSource] 数据已就绪");
                data_cv_.notify_all();
                return true;
            }
        }
        
        // 等待数据到达或超时
        std::unique_lock<std::mutex> lock(data_mutex_);
        if (data_cv_.wait_for(lock, std::chrono::milliseconds(100)) == 
            std::cv_status::timeout) {
            // 检查超时
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(
                now - start).count();
            if (elapsed > timeout_sec) {
                UNICALIB_WARN("[Ros2RealtimeDataSource] 等待超时");
                status_msg_ = "等待超时";
                return false;
            }
        }
    }
    
    return running_;
}

LiDARScanRos Ros2RealtimeDataSource::convert_point_cloud2(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {

    LiDARScanRos scan;
    scan.timestamp = Ros2BagDataSource::stamp_to_sec(msg->header.stamp);
    
    // 转换为 PCL 点云
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    // 过滤点
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
        if (cfg_.filter_nan && (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))) {
            continue;
        }
        
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (dist < cfg_.min_range || dist > cfg_.max_range) {
            continue;
        }
        
        filtered.push_back(pt);
    }
    
    scan.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>(filtered));
    
    return scan;
}

void Ros2RealtimeDataSource::lidar_callback(
    const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    
    auto scan = convert_point_cloud2(msg);
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        lidar_data_[sensor_id].push_back(scan);
        lidar_queues_[sensor_id].push(scan);
        
        // 限制队列大小
        while (lidar_queues_[sensor_id].size() > 1000) {
            lidar_queues_[sensor_id].pop();
        }
        
        UNICALIB_DEBUG("[LidarCallback] 收集 LiDAR {} 数据, 帧数: {}", 
                      sensor_id, lidar_data_[sensor_id].size());
    }
    
    data_cv_.notify_all();
    
    if (data_callback_) {
        data_callback_(sensor_id);
    }
}

void Ros2RealtimeDataSource::camera_callback(
    const std::string& sensor_id,
    const sensor_msgs::msg::Image::SharedPtr msg) {

    CameraFrameRos frame;
    frame.timestamp = Ros2BagDataSource::stamp_to_sec(msg->header.stamp);
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        frame.image = cv_ptr->image.clone();
    } catch (const cv_bridge::Exception& e) {
        UNICALIB_WARN("图像转换失败: {}", e.what());
        frame.image = cv::Mat();
    }
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        camera_data_[sensor_id].push_back(frame);
        camera_queues_[sensor_id].push(frame);
        
        while (camera_queues_[sensor_id].size() > 1000) {
            camera_queues_[sensor_id].pop();
        }
        
        UNICALIB_DEBUG("[CameraCallback] 收集相机 {} 数据, 帧数: {}", 
                      sensor_id, camera_data_[sensor_id].size());
    }
    
    data_cv_.notify_all();
    
    if (data_callback_) {
        data_callback_(sensor_id);
    }
}

void Ros2RealtimeDataSource::imu_callback(
    const std::string& sensor_id,
    const sensor_msgs::msg::Imu::SharedPtr msg) {

    IMUFrameRos frame;
    frame.timestamp = Ros2BagDataSource::stamp_to_sec(msg->header.stamp);
    
    frame.gyro[0] = msg->angular_velocity.x;
    frame.gyro[1] = msg->angular_velocity.y;
    frame.gyro[2] = msg->angular_velocity.z;
    frame.accel[0] = msg->linear_acceleration.x;
    frame.accel[1] = msg->linear_acceleration.y;
    frame.accel[2] = msg->linear_acceleration.z;
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_data_[sensor_id].push_back(frame);
        imu_queues_[sensor_id].push(frame);
        
        while (imu_queues_[sensor_id].size() > 10000) {
            imu_queues_[sensor_id].pop();
        }
        
        UNICALIB_DEBUG("[IMUCallback] 收集 IMU {} 数据, 帧数: {}", 
                      sensor_id, imu_data_[sensor_id].size());
    }
    
    data_cv_.notify_all();
    
    if (data_callback_) {
        data_callback_(sensor_id);
    }
}

std::vector<LiDARScanRos> Ros2RealtimeDataSource::get_lidar_scans(
    const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = lidar_data_.find(sensor_id);
    return (it != lidar_data_.end()) ? it->second : std::vector<LiDARScanRos>();
}

std::vector<CameraFrameRos> Ros2RealtimeDataSource::get_camera_frames(
    const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = camera_data_.find(sensor_id);
    return (it != camera_data_.end()) ? it->second : std::vector<CameraFrameRos>();
}

std::vector<IMUFrameRos> Ros2RealtimeDataSource::get_imu_frames(
    const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = imu_data_.find(sensor_id);
    return (it != imu_data_.end()) ? it->second : std::vector<IMUFrameRos>();
}

std::vector<std::string> Ros2RealtimeDataSource::get_lidar_ids() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> ids;
    for (const auto& [id, _] : lidar_data_) ids.push_back(id);
    return ids;
}

std::vector<std::string> Ros2RealtimeDataSource::get_camera_ids() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> ids;
    for (const auto& [id, _] : camera_data_) ids.push_back(id);
    return ids;
}

std::vector<std::string> Ros2RealtimeDataSource::get_imu_ids() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> ids;
    for (const auto& [id, _] : imu_data_) ids.push_back(id);
    return ids;
}

double Ros2RealtimeDataSource::get_start_time() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double min_time = 1e9;
    for (const auto& [_, data] : lidar_data_) {
        if (!data.empty()) min_time = std::min(min_time, data.front().timestamp);
    }
    for (const auto& [_, data] : camera_data_) {
        if (!data.empty()) min_time = std::min(min_time, data.front().timestamp);
    }
    return (min_time < 1e9) ? min_time : 0.0;
}

double Ros2RealtimeDataSource::get_end_time() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double max_time = 0.0;
    for (const auto& [_, data] : lidar_data_) {
        if (!data.empty()) max_time = std::max(max_time, data.back().timestamp);
    }
    for (const auto& [_, data] : camera_data_) {
        if (!data.empty()) max_time = std::max(max_time, data.back().timestamp);
    }
    return max_time;
}

// ===========================================================================
// 工厂函数
// ===========================================================================

RosDataSourceBase::Ptr create_ros_data_source(const RosDataSourceConfig& cfg) {
    if (cfg.bag_file.empty() && cfg.realtime_mode) {
        // 实时模式
        return std::make_shared<Ros2RealtimeDataSource>(cfg);
    } else if (!cfg.bag_file.empty()) {
        // Bag 模式
        return std::make_shared<Ros2BagDataSource>(cfg);
    } else {
        UNICALIB_ERROR("无效的 ROS2 数据源配置: 未指定 bag 文件或实时模式");
        return nullptr;
    }
}

// ===========================================================================
// UnifiedDataLoader 实现
// ===========================================================================

UnifiedDataLoader::UnifiedDataLoader(const Config& cfg) : cfg_(cfg) {
    status_msg_ = "未加载";
}

UnifiedDataLoader::~UnifiedDataLoader() {
}

bool UnifiedDataLoader::load_from_files() {
    status_msg_ = "从文件加载数据...";
    
    UNICALIB_INFO("[UnifiedDataLoader] 从文件加载 LiDAR: {}", cfg_.lidar_data_dir);
    UNICALIB_INFO("[UnifiedDataLoader] 从文件加载相机: {}", cfg_.camera_images_dir);
    
    try {
        // 加载 LiDAR 点云
        if (!cfg_.lidar_data_dir.empty() && fs::exists(cfg_.lidar_data_dir)) {
            std::vector<fs::path> pcd_files;
            for (const auto& entry : fs::directory_iterator(cfg_.lidar_data_dir)) {
                if (entry.path().extension() == ".pcd") {
                    pcd_files.push_back(entry.path());
                }
            }
            std::sort(pcd_files.begin(), pcd_files.end());
            
            for (const auto& pcd_path : pcd_files) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointXYZI>);
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path.string(), *cloud) == 0) {
                    double ts;
                    try {
                        ts = std::stod(pcd_path.stem().string());
                    } catch (...) {
                        ts = static_cast<double>(file_lidar_data_["lidar_front"].size()) * 0.1;
                    }
                    file_lidar_data_["lidar_front"].push_back(LiDARScanRos(ts, cloud));
                }
            }
            UNICALIB_INFO("  加载 {} 帧 LiDAR 点云", 
                          file_lidar_data_["lidar_front"].size());
        }
        
        // 加载相机图像
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
            UNICALIB_INFO("  加载 {} 帧相机图像", 
                          file_camera_data_["cam_left"].size());
        }
        
        // 加载相机内参
        if (!cfg_.camera_intrinsic_file.empty() && 
            fs::exists(cfg_.camera_intrinsic_file)) {
            try {
                auto intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
                camera_intrinsics_["cam_left"] = intrin;
                UNICALIB_INFO("  加载相机内参: fx={:.1f} fy={:.1f}", 
                              intrin.fx, intrin.fy);
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

bool UnifiedDataLoader::load_from_ros() {
    status_msg_ = "从 ROS2 加载数据...";
    
    UNICALIB_INFO("[UnifiedDataLoader] 从 ROS2 加载数据...");
    UNICALIB_INFO("  Bag 文件: {}", cfg_.ros_config.bag_file);
    UNICALIB_INFO("  实时模式: {}", cfg_.ros_config.realtime_mode);
    
    ros_source_ = create_ros_data_source(cfg_.ros_config);
    if (!ros_source_) {
        status_msg_ = "创建 ROS2 数据源失败";
        return false;
    }
    
    if (!ros_source_->load()) {
        status_msg_ = ros_source_->get_status_message();
        return false;
    }
    
    if (cfg_.ros_config.realtime_mode) {
        // 实时模式: 等待数据就绪
        auto* realtime_source = dynamic_cast<Ros2RealtimeDataSource*>(ros_source_.get());
        if (realtime_source && !realtime_source->wait_for_data(cfg_.ros_config.realtime_timeout)) {
            status_msg_ = "等待 ROS2 数据超时";
            return false;
        }
    }
    
    loaded_ = true;
    status_msg_ = "ROS2 数据加载成功";
    return true;
}

bool UnifiedDataLoader::load() {
    if (cfg_.source_type == SourceType::FILES) {
        return load_from_files();
    } else if (cfg_.source_type == SourceType::ROS2_BAG || 
               cfg_.source_type == SourceType::ROS2_TOPIC) {
        return load_from_ros();
    } else {
        status_msg_ = "未知的数据源类型";
        return false;
    }
}

std::vector<LiDARScanRos> UnifiedDataLoader::get_lidar_scans(
    const std::string& sensor_id) const {
    
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_lidar_data_.find(sensor_id);
        return (it != file_lidar_data_.end()) ? it->second : std::vector<LiDARScanRos>();
    } else {
        return ros_source_ ? ros_source_->get_lidar_scans(sensor_id) : std::vector<LiDARScanRos>();
    }
}

std::vector<CameraFrameRos> UnifiedDataLoader::get_camera_frames(
    const std::string& sensor_id) const {
    
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_camera_data_.find(sensor_id);
        return (it != file_camera_data_.end()) ? it->second : std::vector<CameraFrameRos>();
    } else {
        return ros_source_ ? ros_source_->get_camera_frames(sensor_id) : std::vector<CameraFrameRos>();
    }
}

std::vector<IMUFrameRos> UnifiedDataLoader::get_imu_frames(
    const std::string& sensor_id) const {
    
    if (cfg_.source_type == SourceType::FILES) {
        return {};  // 文件模式暂不支持 IMU
    } else {
        return ros_source_ ? ros_source_->get_imu_frames(sensor_id) : std::vector<IMUFrameRos>();
    }
}

std::vector<LiDARScan> UnifiedDataLoader::to_lidar_scans(
    const std::string& sensor_id) const {
    
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

std::vector<std::pair<double, cv::Mat>> UnifiedDataLoader::to_camera_frames(
    const std::string& sensor_id) const {
    
    std::vector<std::pair<double, cv::Mat>> frames;
    auto ros_frames = get_camera_frames(sensor_id);
    
    for (const auto& ros_frame : ros_frames) {
        frames.emplace_back(ros_frame.timestamp, ros_frame.image);
    }
    
    return frames;
}

IMURawData UnifiedDataLoader::to_imu_raw_data(
    const std::string& sensor_id) const {
    
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

std::optional<CameraIntrinsics> UnifiedDataLoader::get_camera_intrinsics(
    const std::string& sensor_id) const {
    
    auto it = camera_intrinsics_.find(sensor_id);
    if (it != camera_intrinsics_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<std::string> UnifiedDataLoader::get_camera_ids() const {
    if (cfg_.source_type == SourceType::FILES) {
        std::vector<std::string> ids;
        for (const auto& [k, _] : file_camera_data_)
            ids.push_back(k);
        return ids;
    }
    if (ros_source_)
        return ros_source_->get_camera_ids();
    return {};
}

bool UnifiedDataLoader::is_ready() const {
    return loaded_;
}

std::string UnifiedDataLoader::get_status_message() const {
    return status_msg_;
}

}  // namespace ns_unicalib
