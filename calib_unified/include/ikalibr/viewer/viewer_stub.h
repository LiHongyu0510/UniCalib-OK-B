// Stub for viewer: no external tiny-viewer; visualization disabled.
#ifndef IKALIBR_VIEWER_STUB_H
#define IKALIBR_VIEWER_STUB_H

#include "config/configor.h"
#include "ctraj/core/spline_bundle.h"
#include "viewer/viewer_types.h"
#include "util/cloud_define.hpp"
#include "veta/veta.h"
#include "veta/camera/pinhole.h"
// 当 ufomap 可用时使用真实头文件，否则使用桩头文件
#ifdef UNICALIB_NO_UFOMAP
#  include "ufo_stub.h"
#else
#  include "ufo/map/surfel_map.h"
#  include "ufo/map/octree/node.h"
#endif
#include <memory>
#include <string>
#include <map>
#include <list>
#include <vector>
#include <optional>

namespace ns_ikalibr {
struct CalibParamManager;
using CalibParamManagerPtr = std::shared_ptr<CalibParamManager>;
struct PointToSurfelCondition;
struct PointToSurfelCorr;
using PointToSurfelCorrPtr = std::shared_ptr<PointToSurfelCorr>;
struct RGBDFrame;
using RGBDFramePtr = std::shared_ptr<RGBDFrame>;
struct RGBDIntrinsics;
using RGBDIntrinsicsPtr = std::shared_ptr<RGBDIntrinsics>;
struct Feature;
using FeaturePtr = std::shared_ptr<Feature>;
using FeatureVec = std::vector<FeaturePtr>;
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
struct Event;
using EventPtr = std::shared_ptr<Event>;
}  // namespace ns_ikalibr

namespace ns_ikalibr {

constexpr float DefaultPointSize = 0.01f;
constexpr float DefaultLandmarkSize = 0.05f;

class Viewer : public ns_viewer::MultiViewer {
 public:
  using Ptr = std::shared_ptr<Viewer>;
  using Parent = ns_viewer::MultiViewer;
  using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

  inline static const std::string VIEW_SENSORS = "VIEW_SENSORS";
  inline static const std::string VIEW_SPLINE = "VIEW_SPLINE";
  inline static const std::string VIEW_MAP = "VIEW_MAP";
  inline static const std::string VIEW_ASSOCIATION = "VIEW_ASSOCIATION";

  explicit Viewer(CalibParamManagerPtr, const SplineBundleType::Ptr&) {}
  static Ptr Create(const CalibParamManagerPtr& p, const SplineBundleType::Ptr& s) {
    return std::make_shared<Viewer>(p, s);
  }

  Viewer& FillEmptyViews(const std::string&) { return *this; }
  Viewer& UpdateSensorViewer() { return *this; }
  Viewer& UpdateSplineViewer(double = 0.005) { return *this; }
  bool IsActive() const { return false; }

  Viewer& AddAlignedCloud(const IKalibrPointCloud::Ptr&, const std::string&,
                        const Eigen::Vector3f& = {0, 0, 1}, float = DefaultPointSize) {
    return *this;
  }
  Viewer& AddCloud(const IKalibrPointCloud::Ptr&, const std::string&,
                 const ns_viewer::Colour&, float = DefaultPointSize) { return *this; }
  Viewer& AddCloud(const IKalibrPointCloud::Ptr&, const std::string&, float = DefaultPointSize) {
    return *this;
  }
  Viewer& AddStarMarkCloud(const IKalibrPointCloud::Ptr&, const std::string&,
                         float = DefaultLandmarkSize) { return *this; }
  Viewer& ClearViewer(const std::string&) { return *this; }
  Viewer& PopBackEntity(const std::string&) { return *this; }

  Viewer& AddSurfelMap(const ufo::map::SurfelMap&, const PointToSurfelCondition&,
                     const std::string&) { return *this; }
  Viewer& AddPointToSurfel(const ufo::map::SurfelMap&,
                         const std::map<std::string, std::vector<PointToSurfelCorrPtr>>&,
                         const std::string&) { return *this; }
  ns_viewer::Entity::Ptr Gravity() const { return nullptr; }

  Viewer& AddVeta(const ns_veta::Veta::Ptr&, const std::string&,
                const std::optional<ns_viewer::Colour>& = ns_viewer::Colour::Blue(),
                const std::optional<ns_viewer::Colour>& = {}) { return *this; }
  Viewer& AddEntityLocal(const std::vector<ns_viewer::Entity::Ptr>&, const std::string&) {
    return *this;
  }
  void SetNewSpline(const SplineBundleType::Ptr&) {}
  Viewer& AddObjEntity(const std::string&, const std::string&) { return *this; }

  Viewer& AddRGBDFrame(const RGBDFramePtr&, const RGBDIntrinsicsPtr&, const std::string&,
                    bool, float) { return *this; }
  Viewer& AddEventFeatTracking(const std::map<int, FeatureVec>&,
                               const ns_veta::PinholeIntrinsic::Ptr&, float, float,
                               const std::string&, const std::pair<float, float>& = {0.01f, 2.0f}) {
    return *this;
  }
  Viewer& AddEventFeatTracking(const FeatureVec&, float, const std::string&,
                               const std::pair<float, float>& = {0.01f, 2.0f}) { return *this; }
  Viewer& AddSpatioTemporalTrace(const std::vector<Eigen::Vector3d>&, float, const std::string&,
                                 float = 0.5f, const ns_viewer::Colour& = ns_viewer::Colour::Blue(),
                                 const std::pair<float, float>& = {0.01f, 2.0f}) { return *this; }
  Viewer& AddEventData(const std::vector<EventArrayPtr>::const_iterator&,
                       const std::vector<EventArrayPtr>::const_iterator&, float, const std::string&,
                       const std::pair<float, float>& = {0.01f, 2.0f}, float = 1.0f) {
    return *this;
  }
  Viewer& AddEventData(const EventArrayPtr&, float, const std::string&,
                      const std::pair<float, float>& = {0.01f, 2.0f},
                      const std::optional<ns_viewer::Colour>& = {}, float = 1.0f) {
    return *this;
  }
  Viewer& AddEventData(const std::list<EventPtr>&, float, const std::string&,
                      const std::pair<float, float>& = {0.01f, 2.0f},
                      const std::optional<ns_viewer::Colour>& = {}, float = 1.0f) {
    return *this;
  }

  std::vector<std::size_t> AddEntity(const std::vector<ns_viewer::Entity::Ptr>&) { return {}; }
  std::vector<std::size_t> AddEntity(const std::vector<ns_viewer::Entity::Ptr>&,
                                    const std::string&) { return {}; }
  void RemoveEntity(const std::vector<std::size_t>&, const std::string&) override {}
  void RemoveEntity(std::size_t, const std::string&) override {}
  void RunInMultiThread() override {}
};

}  // namespace ns_ikalibr

#endif  // IKALIBR_VIEWER_STUB_H
