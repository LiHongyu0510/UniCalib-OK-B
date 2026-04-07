/**
 * UniCalib Unified — ROS2 数据源抽象
 * 
 * 支持:
 *   - ROS2 bag 文件读取 (sqlite3 格式)
 *   - ROS2 话题实时订阅 (rclcpp)
 *   - 同步多话题数据 (时间戳对齐)
 * 
 * 设计原则:
 *   - 与现有文件数据加载并存
 *   - 最小化侵入性修改
 *   - 可选编译 (UNICALIB_WITH_ROS2 宏控制)
 */

#pragma once

#include "unicalib/common/sensor_types.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/calib_param.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#endif

// Forward declarations for ROS2 types
namespace rclcpp {
    class Node;
    class Executor;
    class SubscriptionBase;
}

namespace rosbag2_cpp {
namespace readers {
    class SequentialReader;
}
}

namespace ns_unicalib {

// ===========================================================================
// LiDAR 扫描数据 (带时间戳)
// ===========================================================================
struct LiDARScanRos {
    double timestamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    
    LiDARScanRos() : timestamp(0.0), cloud(nullptr) {}
    LiDARScanRos(double ts, pcl::PointCloud<pcl::PointXYZI>::Ptr c) 
        : timestamp(ts), cloud(c) {}
};

// ===========================================================================
// 相机帧数据 (带时间戳)
// ===========================================================================
struct CameraFrameRos {
    double timestamp;
    cv::Mat image;
    
    CameraFrameRos() : timestamp(0.0) {}
    CameraFrameRos(double ts, const cv::Mat& img) : timestamp(ts), image(img.clone()) {}
};

// ===========================================================================
// IMU 数据 (带时间戳)
// ===========================================================================
struct IMUFrameRos {
    double timestamp;
    double gyro[3];   // rad/s
    double accel[3];  // m/s²
    
    IMUFrameRos() : timestamp(0.0) {
        gyro[0] = gyro[1] = gyro[2] = 0.0;
        accel[0] = accel[1] = accel[2] = 0.0;
    }
};

// ===========================================================================
// ROS2 数据源配置
// ===========================================================================
struct RosDataSourceConfig {
    // bag 文件路径
    std::string bag_file;
    
    // 话题映射: sensor_id -> topic name（多传感器时使用）
    std::map<std::string, std::string> lidar_topics;
    std::map<std::string, std::string> camera_topics;
    std::map<std::string, std::string> imu_topics;
    
    // 单话题覆盖（可选；未设置时从 lidar_topics/camera_topics/imu_topics 取，或由 sensors 配置默认）
    std::string lidar_ros2_topic;
    std::string camera_ros2_topic;
    std::string imu_ros2_topic;
    
    // 时间窗口 (秒, 0 = 全部数据)
    double time_window_start = 0.0;
    double time_window_end = 0.0;
    
    // 采样间隔 (秒, 0 = 不跳过)
    double sample_interval = 0.0;
    
    // 最大帧数 (0 = 无限制)
    size_t max_frames = 0;
    
    // 以配置文件为准：为 true 时，配置的话题在 bag 中无数据则加载失败，不自动回退到其他话题
    bool strict_topic_match = true;
    
    // 实时模式 (订阅话题而非播放bag)
    bool realtime_mode = false;
    
    // 实时模式超时 (秒)
    double realtime_timeout = 30.0;
    
    // LiDAR 点云滤波
    double min_range = 0.5;    // 最小距离
    double max_range = 100.0;  // 最大距离
    bool   filter_nan = true;  // 过滤 NaN 点
};

// ===========================================================================
// ROS2 数据源基类
// ===========================================================================
class RosDataSourceBase {
public:
    using Ptr = std::shared_ptr<RosDataSourceBase>;
    
    explicit RosDataSourceBase(const RosDataSourceConfig& cfg) : cfg_(cfg) {}
    virtual ~RosDataSourceBase() = default;
    
    // 加载数据
    virtual bool load() = 0;
    
    // 获取数据
    virtual std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id) const = 0;
    virtual std::vector<CameraFrameRos> get_camera_frames(const std::string& sensor_id) const = 0;
    virtual std::vector<IMUFrameRos> get_imu_frames(const std::string& sensor_id) const = 0;
    
    // 获取可用传感器 ID
    virtual std::vector<std::string> get_lidar_ids() const = 0;
    virtual std::vector<std::string> get_camera_ids() const = 0;
    virtual std::vector<std::string> get_imu_ids() const = 0;
    
    // 时间戳范围
    virtual double get_start_time() const = 0;
    virtual double get_end_time() const = 0;
    
    // 状态
    virtual bool is_ready() const = 0;
    virtual std::string get_status_message() const = 0;

protected:
    RosDataSourceConfig cfg_;
};

// ===========================================================================
// ROS2 Bag 文件数据源
// ===========================================================================
class Ros2BagDataSource : public RosDataSourceBase {
public:
    using Ptr = std::shared_ptr<Ros2BagDataSource>;
    
    explicit Ros2BagDataSource(const RosDataSourceConfig& cfg);
    ~Ros2BagDataSource() override;
    
    bool load() override;
    
    std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id) const override;
    std::vector<CameraFrameRos> get_camera_frames(const std::string& sensor_id) const override;
    std::vector<IMUFrameRos> get_imu_frames(const std::string& sensor_id) const override;
    
    std::vector<std::string> get_lidar_ids() const override;
    std::vector<std::string> get_camera_ids() const override;
    std::vector<std::string> get_imu_ids() const override;
    
    double get_start_time() const override { return start_time_; }
    double get_end_time() const override { return end_time_; }
    
    bool is_ready() const override { return loaded_; }
    std::string get_status_message() const override { return status_msg_; }

    // 从 bag metadata.yaml 自动推断话题名称
    struct TopicMapping {
        std::string lidar_topic;
        std::string camera_topic;
        std::string imu_topic;
    };
    TopicMapping get_topic_mapping() const { return topic_mapping_; }

    // 时间戳转换 (供 Ros2RealtimeDataSource 等复用)
    static double stamp_to_sec(const builtin_interfaces::msg::Time& stamp);

private:
    // 内部数据存储
    std::map<std::string, std::vector<LiDARScanRos>> lidar_data_;
    std::map<std::string, std::vector<CameraFrameRos>> camera_data_;
    std::map<std::string, std::vector<IMUFrameRos>> imu_data_;
    
    double start_time_ = 0.0;
    double end_time_ = 0.0;
    bool loaded_ = false;
    std::string status_msg_;
    
    // 话题映射（从 metadata.yaml 自动推断或手动配置）
    TopicMapping topic_mapping_;
    
    // 从 metadata.yaml 解析的话题信息（用于诊断日志与 fallback）
    std::map<std::string, size_t> bag_topic_msg_counts_;
    std::map<std::string, std::string> bag_topic_types_;
    
    // sensor_id / 话题名 -> 实际话题名（用于“选择使用”：配置或默认选中的话题）
    // get_*_frames(sensor_id) 先查此映射，再查 *_data_ 中 key 为话题名的数据
    std::map<std::string, std::string> effective_lidar_topic_;
    std::map<std::string, std::string> effective_camera_topic_;
    std::map<std::string, std::string> effective_imu_topic_;
    
    // Bag 读取器
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> bag_reader_;
    
    // 解析 PointCloud2 消息
    LiDARScanRos parse_point_cloud2(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& msg);
    
    // 解析 Image 消息
    CameraFrameRos parse_image(const std::shared_ptr<sensor_msgs::msg::Image>& msg);
    
    // 解析 IMU 消息
    IMUFrameRos parse_imu(const std::shared_ptr<sensor_msgs::msg::Imu>& msg);

    // 从 bag metadata.yaml 读取话题映射
    bool read_metadata_yaml(const std::string& bag_file);
    
    // 智能话题匹配
    void auto_detect_topics(const std::map<std::string, size_t>& topic_counts);
    std::string find_best_lidar_topic(const std::map<std::string, size_t>& topic_msg_counts);
    std::string find_best_camera_topic(const std::map<std::string, size_t>& topic_msg_counts);
    std::string find_best_imu_topic(const std::map<std::string, size_t>& topic_msg_counts);
};

// ===========================================================================
// ROS2 实时话题数据源 (订阅话题)
// ===========================================================================
class Ros2RealtimeDataSource : public RosDataSourceBase {
public:
    using Ptr = std::shared_ptr<Ros2RealtimeDataSource>;
    using DataCallback = std::function<void(const std::string& sensor_id)>;
    
    explicit Ros2RealtimeDataSource(const RosDataSourceConfig& cfg);
    ~Ros2RealtimeDataSource() override;
    
    bool load() override;
    
    // 启动/停止订阅
    bool start();
    void stop();
    
    // 等待数据就绪
    bool wait_for_data(double timeout_sec = 10.0);
    
    // 设置数据到达回调
    void set_data_callback(DataCallback cb) { data_callback_ = cb; }
    
    std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id) const override;
    std::vector<CameraFrameRos> get_camera_frames(const std::string& sensor_id) const override;
    std::vector<IMUFrameRos> get_imu_frames(const std::string& sensor_id) const override;
    
    std::vector<std::string> get_lidar_ids() const override;
    std::vector<std::string> get_camera_ids() const override;
    std::vector<std::string> get_imu_ids() const override;
    
    double get_start_time() const override;
    double get_end_time() const override;
    
    bool is_ready() const override { return running_; }
    std::string get_status_message() const override { return status_msg_; }

private:
    // ROS2 节点
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    
    // 订阅者 (存储以保持订阅活跃)
    std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
    
    // 数据缓存 (带锁)
    mutable std::mutex data_mutex_;
    std::map<std::string, std::queue<LiDARScanRos>> lidar_queues_;
    std::map<std::string, std::queue<CameraFrameRos>> camera_queues_;
    std::map<std::string, std::queue<IMUFrameRos>> imu_queues_;
    
    // 已收集的完整数据
    std::map<std::string, std::vector<LiDARScanRos>> lidar_data_;
    std::map<std::string, std::vector<CameraFrameRos>> camera_data_;
    std::map<std::string, std::vector<IMUFrameRos>> imu_data_;
    
    // 状态
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::string status_msg_;
    
    // 数据到达回调
    DataCallback data_callback_;
    
    // 条件变量 (用于等待数据)
    mutable std::condition_variable data_cv_;
    
    // 话题回调
    void lidar_callback(const std::string& sensor_id, 
                       const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void camera_callback(const std::string& sensor_id,
                        const sensor_msgs::msg::Image::SharedPtr msg);
    void imu_callback(const std::string& sensor_id,
                     const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // 点云转换
    LiDARScanRos convert_point_cloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
};

// ===========================================================================
// 工厂函数: 根据配置创建数据源
// ===========================================================================
RosDataSourceBase::Ptr create_ros_data_source(const RosDataSourceConfig& cfg);

// ===========================================================================
// 统一数据加载接口 (文件 + ROS2)
// ===========================================================================
class UnifiedDataLoader {
public:
    enum class SourceType {
        FILES,      // PCD 文件 + 图像文件
        ROS2_BAG,   // ROS2 bag 文件
        ROS2_TOPIC  // ROS2 实时话题
    };
    
    struct Config {
        SourceType source_type = SourceType::FILES;
        
        // 文件模式
        std::string lidar_data_dir;
        std::string camera_images_dir;
        std::string camera_intrinsic_file;
        
        // ROS2 模式
        RosDataSourceConfig ros_config;
        
        // 通用参数
        size_t max_frames = 100;
        double sample_interval = 0.0;
    };
    
    explicit UnifiedDataLoader(const Config& cfg);
    ~UnifiedDataLoader();
    
    // 加载数据
    bool load();
    
    // 获取数据 (统一接口)
    std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id = "lidar_front") const;
    std::vector<CameraFrameRos> get_camera_frames(const std::string& sensor_id = "cam_left") const;
    std::vector<IMUFrameRos> get_imu_frames(const std::string& sensor_id = "imu_0") const;
    
    // 转换为标定器所需格式
    std::vector<LiDARScan> to_lidar_scans(const std::string& sensor_id = "lidar_front") const;
    std::vector<std::pair<double, cv::Mat>> to_camera_frames(const std::string& sensor_id = "cam_left") const;
    IMURawData to_imu_raw_data(const std::string& sensor_id = "imu_0") const;
    
    // 获取内参 (若可用)
    std::optional<CameraIntrinsics> get_camera_intrinsics(const std::string& sensor_id = "cam_left") const;
    
    /** 获取已加载的相机 ID 列表（ROS2 bag 下为话题名 + 映射 id，文件模式为 data key） */
    std::vector<std::string> get_camera_ids() const;
    
    // 状态
    bool is_ready() const;
    std::string get_status_message() const;
    SourceType get_source_type() const { return cfg_.source_type; }

private:
    Config cfg_;
    
    // 数据源
    RosDataSourceBase::Ptr ros_source_;
    
    // 从文件加载的数据
    std::map<std::string, std::vector<LiDARScanRos>> file_lidar_data_;
    std::map<std::string, std::vector<CameraFrameRos>> file_camera_data_;
    
    // 内参
    std::map<std::string, CameraIntrinsics> camera_intrinsics_;
    
    bool loaded_ = false;
    std::string status_msg_;
    
    // 从文件加载
    bool load_from_files();
    
    // 从 ROS2 加载
    bool load_from_ros();
};

}  // namespace ns_unicalib
