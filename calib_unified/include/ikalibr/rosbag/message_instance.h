// ROS2 兼容桩：原为 ROS1 rosbag::MessageInstance，现提供最小接口使头文件可编译。
// 实际从 bag 读数据需迁移到 rosbag2 API。
#pragma once
#include <memory>
#include <string>
namespace rosbag {
class MessageInstance {
public:
    std::string getTopic() const { return {}; }
    template <typename T>
    std::shared_ptr<T> instantiate() const { return nullptr; }
};
}
