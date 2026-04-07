// Stub when velodyne_msgs package is not available (e.g. ROS2-only build).
// Real velodyne support: install velodyne_msgs and build with -DIKALIBR_USE_VELODYNE_MSGS.
#ifndef IKALIBR_VELODYNE_SCAN_STUB_H
#define IKALIBR_VELODYNE_SCAN_STUB_H

#include <memory>
#include <vector>
#include <cstdint>

namespace velodyne_msgs {

struct Stamp {
  int32_t sec = 0;
  uint32_t nanosec = 0;
  bool isZero() const { return sec == 0 && nanosec == 0; }
  double toSec() const { return static_cast<double>(sec) + 1e-9 * nanosec; }
};

struct Header {
  Stamp stamp;
};

struct VelodyneScan {
  Header header;
  std::vector<uint8_t> packets;  // minimal placeholder
};

using ConstPtr = std::shared_ptr<const VelodyneScan>;

}  // namespace velodyne_msgs

#endif  // IKALIBR_VELODYNE_SCAN_STUB_H
