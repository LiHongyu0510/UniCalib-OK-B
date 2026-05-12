// Stub for ROS2 build: provide minimal rosbag API so code compiles.
// Real bag loading requires ROS1 rosbag or a rosbag2 port (see calib_data_manager.cpp).
#ifndef IKALIBR_ROSBAG_BAG_STUB_H
#define IKALIBR_ROSBAG_BAG_STUB_H

#include <string>
#include <vector>
#include <stdexcept>
// ros::Time is used in MessageNumInTopic signature
#include "ros/ros.h"

namespace rosbag {

struct TopicQuery {
  explicit TopicQuery(const std::string& topic) : topics_({topic}) {}
  explicit TopicQuery(const std::vector<std::string>& topics) : topics_(topics) {}
  std::vector<std::string> topics_;
};

enum BagMode { Read = 0, Write = 1 };

// Minimal Bag stub: methods throw at runtime to indicate missing rosbag2 support
class Bag {
public:
    Bag() = default;
    ~Bag() { close(); }

    void open(const std::string& path, BagMode mode) {
        (void)path; (void)mode;
        throw std::runtime_error(
            "rosbag::Bag::open() is not supported in ROS2 build. "
            "Use rosbag2 API or convert data to a supported format.");
    }

    void close() {}

    bool isOpen() const { return false; }

    void write(const std::string& topic, const ros::Time& time, const void* msg) {
        (void)topic; (void)time; (void)msg;
        throw std::runtime_error("rosbag::Bag::write() not supported in ROS2 build.");
    }
};

class View;  // defined in rosbag/view.h

}  // namespace rosbag

#endif  // IKALIBR_ROSBAG_BAG_STUB_H
