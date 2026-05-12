#ifndef VETA_STUB_VETA_H
#define VETA_STUB_VETA_H

#include "veta/type_def.hpp"
#include "veta/landmark.h"
#include "veta/camera/pinhole.h"
#include <memory>
#include <map>
#include <string>

namespace ns_veta {

struct View {
  IndexT viewId = UndefinedIndexT;
  IndexT poseId = UndefinedIndexT;
  IndexT intrinsicId = UndefinedIndexT;
  int imgWidth = 0;
  int imgHeight = 0;
  double timestamp = 0.0;
  std::string imageName;

  View() = default;
  View(double ts, IndexT vid, IndexT iid, IndexT pid, int w, int h)
      : viewId(vid), poseId(pid), intrinsicId(iid), imgWidth(w), imgHeight(h), timestamp(ts) {}

  static std::shared_ptr<View> Create(double ts, IndexT vid, IndexT iid, IndexT pid, int w, int h) {
    return std::make_shared<View>(ts, vid, iid, pid, w, h);
  }
};
using ViewPtr = std::shared_ptr<View>;

struct Veta {
  using Ptr = std::shared_ptr<Veta>;
  std::map<IndexT, Landmark> landmarks;
  std::map<IndexT, Landmark>& structure;
  std::map<IndexT, Posed> poses;
  std::map<IndexT, ViewPtr> views;
  std::map<IndexT, PinholeIntrinsicPtr> intrinsics;
  std::map<std::string, IndexT> viewIdToName;
  Veta() : structure(landmarks) {}
  static Ptr Create() { return std::make_shared<Veta>(); }
};

using VetaPtr = Veta::Ptr;

enum SaveFlags {
  ALL = 0,
  STRUCTURE = 1,
  POSES = 2,
  VIEWS = 4
};

inline bool Save(const Veta&, const std::string&, int flags = ALL) {
  (void)flags;
  return false;
}

}  // namespace ns_veta

#endif
