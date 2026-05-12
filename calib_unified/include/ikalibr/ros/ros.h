// ROS2 compatibility shim: provide ros:: namespace for iKalibr
// When UNICALIB_WITH_ROS2=1, uses real rclcpp types.
// When disabled, provides lightweight stubs so ikalibr sources still compile.
#ifndef IKALIBR_ROS2_ROS_ROS_H
#define IKALIBR_ROS2_ROS_ROS_H

#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2

// ─── Full ROS2 implementation ──────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
#include <string>

namespace ros {

using Time = rclcpp::Time;

struct Duration {
  rclcpp::Duration d_;
  Duration() : d_(0, 0) {}
  explicit Duration(double seconds) : d_(std::chrono::duration<double>(seconds)) {}
  operator rclcpp::Duration() const { return d_; }
};

namespace detail {
inline std::shared_ptr<rclcpp::Node>& global_node() {
  static std::shared_ptr<rclcpp::Node> node;
  return node;
}
}  // namespace detail

inline void init(int argc, char** argv, const std::string& node_name) {
  if (!rclcpp::ok()) rclcpp::init(argc, argv);
  if (!detail::global_node())
    detail::global_node() = rclcpp::Node::make_shared(node_name);
}
inline bool ok() { return rclcpp::ok(); }
inline void spin(std::shared_ptr<rclcpp::Node> node = nullptr) {
  if (node) rclcpp::spin(node);
  else if (detail::global_node()) rclcpp::spin(detail::global_node());
}
inline void shutdown() { rclcpp::shutdown(); }

namespace param {
namespace detail {
inline rclcpp::Node* param_node() {
  auto& n = ::ros::detail::global_node();
  return n ? n.get() : nullptr;
}
}
inline void set_param_node(rclcpp::Node* /*node*/) {}
template <typename T>
bool get(const std::string& name, T& value) {
  rclcpp::Node* node = detail::param_node();
  if (!node) return false;
  std::string key = name;
  if (!key.empty() && key[0] == '/') key = key.substr(1);
  auto slash = key.find('/');
  if (slash != std::string::npos) key = key.substr(slash + 1);
  try {
    if (!node->has_parameter(key)) node->declare_parameter<T>(key, T{});
    value = node->get_parameter(key).get_value<T>();
    return true;
  } catch (...) { return false; }
}
}  // namespace param

namespace package {
inline std::string getPath(const std::string& name) {
  try { return ament_index_cpp::get_package_share_directory(name); }
  catch (...) { return ""; }
}
}  // namespace package

}  // namespace ros

#else  // No ROS2 — lightweight stub

// ─── Stub implementation (no rclcpp dependency) ───────────────────────────
#include <chrono>
#include <string>
#include <memory>
#include <cstdint>

namespace ros {

// Minimal Time stub (nanoseconds since epoch)
struct Time {
  int64_t nanoseconds_{0};
  Time() = default;
  explicit Time(double seconds)
      : nanoseconds_(static_cast<int64_t>(seconds * 1e9)) {}
  Time(int32_t sec, uint32_t nsec)
      : nanoseconds_(static_cast<int64_t>(sec) * 1000000000LL + nsec) {}
  double seconds() const { return nanoseconds_ * 1e-9; }
  int32_t sec()  const { return static_cast<int32_t>(nanoseconds_ / 1000000000LL); }
  uint32_t nanosec() const { return static_cast<uint32_t>(nanoseconds_ % 1000000000LL); }
  bool operator<(const Time& o) const { return nanoseconds_ < o.nanoseconds_; }
  bool operator>(const Time& o) const { return nanoseconds_ > o.nanoseconds_; }
  bool operator==(const Time& o) const { return nanoseconds_ == o.nanoseconds_; }
  double operator-(const Time& o) const { return (nanoseconds_ - o.nanoseconds_) * 1e-9; }
};

struct Duration {
  int64_t nanoseconds_{0};
  Duration() = default;
  explicit Duration(double seconds)
      : nanoseconds_(static_cast<int64_t>(seconds * 1e9)) {}
  double seconds() const { return nanoseconds_ * 1e-9; }
};

inline void init(int /*argc*/, char** /*argv*/, const std::string& /*name*/) {}
inline bool ok() { return true; }
inline void shutdown() {}

namespace param {
inline void set_param_node(void* /*node*/) {}
template <typename T>
bool get(const std::string& /*name*/, T& /*value*/) { return false; }
}  // namespace param

namespace package {
inline std::string getPath(const std::string& /*name*/) { return ""; }
}  // namespace package

}  // namespace ros

#endif  // UNICALIB_WITH_ROS2

#endif  // IKALIBR_ROS2_ROS_ROS_H
