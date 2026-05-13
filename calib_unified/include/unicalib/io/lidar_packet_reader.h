#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>
#include <vector>

namespace ns_unicalib {

struct DecodedLidarFrame {
    double timestamp_sec = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

/**
 * 从 sensor_decode/lidar_decode 中提取的“纯读取/解码”模块：
 * - 不依赖 ROS2 / rclcpp / sensor_msgs
 * - Livox/Hesai：单文件字节流
 * - Blindspot：AACC 分隔的自定义帧流
 * - RoboSense RS-Helios：MSOP + DIFOP（可先 decode_file 自动按文件名推断 DIFOP）
 * - 现已透明支持 .gz / .pcd.gz 压缩文件（内部 zlib 解压后走原有解码路径）
 */
class LidarPacketReader {
public:
    // 依次尝试：Livox/Hesai -> Blindspot(AACC) -> RS MSOP
    static bool decode_file(const std::string& file_path,
                            std::vector<DecodedLidarFrame>& frames,
                            std::size_t max_frames = 0,
                            double blindspot_timestamp_unit_scale = 1e-6);

    // 显式指定成对 MSOP / DIFOP（与 sensor_decode 工具一致）
    static bool decode_rs_msop_difop(const std::string& msop_path,
                                     const std::string& difop_path,
                                     std::vector<DecodedLidarFrame>& frames,
                                     std::size_t max_frames = 0);
};

}  // namespace ns_unicalib

