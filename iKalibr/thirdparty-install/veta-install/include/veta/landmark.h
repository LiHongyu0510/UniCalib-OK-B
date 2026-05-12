#ifndef VETA_STUB_LANDMARK_H
#define VETA_STUB_LANDMARK_H

#include "veta/type_def.hpp"
#include <map>

namespace ns_veta {

struct Observation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2d x;
  IndexT id{};
  Observation() = default;
  Observation(const Vec2d& pt, IndexT idx = 0) : x(pt), id(idx) {}
};

struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3d X;
  std::map<IndexT, Observation> obs;
  Landmark() = default;
  Landmark(const Vec3d& pos, std::map<IndexT, Observation> observations = {})
      : X(pos), obs(std::move(observations)) {}
};

}  // namespace ns_veta

#endif
