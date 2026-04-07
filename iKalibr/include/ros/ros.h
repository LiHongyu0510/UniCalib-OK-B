// ROS2 compatibility shim: provide ros:: namespace for iKalibr (build with ROS2 Humble)
#ifndef IKALIBR_ROS2_ROS_ROS_H
#define IKALIBR_ROS2_ROS_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
#include <string>

namespace ros {

using Time = rclcpp::Time;

// Wrapper so that ros::Duration(1.5) works (rclcpp::Duration uses chrono, not double)
struct Duration {
  rclcpp::Duration d_;
  Duration() : d_(0, 0) {}
  explicit Duration(double seconds) : d_(std::chrono::duration<double>(seconds)) {}
  operator rclcpp::Duration() const { return d_; }
};

namespace detail {
// Global rclcpp node used for parameter access
inline std::shared_ptr<rclcpp::Node>& global_node() {
  static std::shared_ptr<rclcpp::Node> node;
  return node;
}
}  // namespace detail

// ros::init() equivalent: initializes rclcpp and creates a global node
inline void init(int argc, char** argv, const std::string& node_name) {
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  if (!detail::global_node()) {
    detail::global_node() = rclcpp::Node::make_shared(node_name);
  }
}

inline bool ok() {
  return rclcpp::ok();
}

inline void spin(std::shared_ptr<rclcpp::Node> node = nullptr) {
  if (node) {
    rclcpp::spin(node);
  } else if (detail::global_node()) {
    rclcpp::spin(detail::global_node());
  }
}

inline void shutdown() {
  rclcpp::shutdown();
}

namespace param {

namespace detail {
inline rclcpp::Node* param_node() {
  auto& n = ::ros::detail::global_node();
  return n ? n.get() : nullptr;
}
}  // namespace detail

inline void set_param_node(rclcpp::Node* node) {
  (void)node;  // no-op; use ros::init() to create the global node
}

template <typename T>
bool get(const std::string& name, T& value) {
  rclcpp::Node* node = detail::param_node();
  if (!node) return false;
  // Strip leading '/' and node-name prefix if present (ROS1 compat)
  std::string key = name;
  if (!key.empty() && key[0] == '/') key = key.substr(1);
  auto slash = key.find('/');
  if (slash != std::string::npos) key = key.substr(slash + 1);
  try {
    if (!node->has_parameter(key)) {
      node->declare_parameter<T>(key, T{});
    }
    value = node->get_parameter(key).get_value<T>();
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace param

namespace package {

inline std::string getPath(const std::string& name) {
  try {
    return ament_index_cpp::get_package_share_directory(name);
  } catch (const std::exception&) {
    return "";
  }
}

}  // namespace package

}  // namespace ros

#endif  // IKALIBR_ROS2_ROS_ROS_H
