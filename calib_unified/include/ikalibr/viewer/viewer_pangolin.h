// iKalibr: Pangolin-backed Viewer (replaces tiny-viewer).
#ifndef IKALIBR_VIEWER_PANGOLIN_H
#define IKALIBR_VIEWER_PANGOLIN_H

#include "config/configor.h"
#include "ctraj/core/spline_bundle.h"
#include "viewer/viewer_types.h"
#include "util/cloud_define.hpp"
#include "veta/veta.h"
#include "veta/camera/pinhole.h"
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
#include <atomic>
#include <mutex>

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

// Pangolin-backed multi-viewer: window + 4 views, draw entities.
class MultiViewerPangolin : public ns_viewer::MultiViewer {
 public:
  explicit MultiViewerPangolin(const ns_viewer::MultiViewerConfigor& config);
  ~MultiViewerPangolin() override;
  std::vector<std::size_t> AddEntity(const std::vector<ns_viewer::Entity::Ptr>&) override;
  std::vector<std::size_t> AddEntity(const std::vector<ns_viewer::Entity::Ptr>&,
                                     const std::string&) override;
  void RemoveEntity(const std::vector<std::size_t>&, const std::string&) override;
  void RemoveEntity(std::size_t, const std::string&) override;
  void RunInMultiThread() override;
  bool IsActive() const { return _active; }

 protected:
  ns_viewer::MultiViewerConfigor _config;
  std::map<std::string, std::vector<ns_viewer::Entity::Ptr>> _store;
  std::map<std::string, std::vector<std::size_t>> _handles;
  std::size_t _nextHandle = 0;
  std::mutex _mutex;
  std::atomic<bool> _active{false};
};

class Viewer : public MultiViewerPangolin {
 public:
  using Ptr = std::shared_ptr<Viewer>;
  using Parent = MultiViewerPangolin;
  using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

  inline static const std::string VIEW_SENSORS = "VIEW_SENSORS";
  inline static const std::string VIEW_SPLINE = "VIEW_SPLINE";
  inline static const std::string VIEW_MAP = "VIEW_MAP";
  inline static const std::string VIEW_ASSOCIATION = "VIEW_ASSOCIATION";

  explicit Viewer(CalibParamManagerPtr parMagr, const SplineBundleType::Ptr& splines);
  static Ptr Create(const CalibParamManagerPtr& p, const SplineBundleType::Ptr& s) {
    return std::make_shared<Viewer>(p, s);
  }

  Viewer& FillEmptyViews(const std::string& objPath);
  Viewer& UpdateSensorViewer();
  Viewer& UpdateSplineViewer(double dt = 0.005);
  bool IsActive() const { return MultiViewerPangolin::IsActive(); }

  Viewer& AddAlignedCloud(const IKalibrPointCloud::Ptr&, const std::string&,
                          const Eigen::Vector3f& = {0, 0, 1}, float = DefaultPointSize);
  Viewer& AddCloud(const IKalibrPointCloud::Ptr&, const std::string&,
                   const ns_viewer::Colour&, float = DefaultPointSize);
  Viewer& AddCloud(const IKalibrPointCloud::Ptr&, const std::string&, float = DefaultPointSize);
  Viewer& AddStarMarkCloud(const IKalibrPointCloud::Ptr&, const std::string&,
                           float = DefaultLandmarkSize);
  Viewer& ClearViewer(const std::string&);
  Viewer& PopBackEntity(const std::string&);

  Viewer& AddSurfelMap(const ufo::map::SurfelMap&, const PointToSurfelCondition&,
                       const std::string&);
  Viewer& AddPointToSurfel(const ufo::map::SurfelMap&,
                           const std::map<std::string, std::vector<PointToSurfelCorrPtr>>&,
                           const std::string&);
  ns_viewer::Entity::Ptr Gravity() const;

  Viewer& AddVeta(const ns_veta::Veta::Ptr&, const std::string&,
                  const std::optional<ns_viewer::Colour>& = ns_viewer::Colour::Blue(),
                  const std::optional<ns_viewer::Colour>& = {});
  Viewer& AddEntityLocal(const std::vector<ns_viewer::Entity::Ptr>&, const std::string&);
  void SetNewSpline(const SplineBundleType::Ptr&);
  Viewer& AddObjEntity(const std::string& objPath, const std::string& view);

  Viewer& AddRGBDFrame(const RGBDFramePtr&, const RGBDIntrinsicsPtr&, const std::string&,
                      bool, float);
  Viewer& AddEventFeatTracking(const std::map<int, FeatureVec>&,
                              const ns_veta::PinholeIntrinsic::Ptr&, float, float,
                              const std::string&, const std::pair<float, float>& = {0.01f, 2.0f});
  Viewer& AddEventFeatTracking(const FeatureVec&, float, const std::string&,
                              const std::pair<float, float>& = {0.01f, 2.0f});
  Viewer& AddSpatioTemporalTrace(const std::vector<Eigen::Vector3d>&, float, const std::string&,
                                 float = 0.5f, const ns_viewer::Colour& = ns_viewer::Colour::Blue(),
                                 const std::pair<float, float>& = {0.01f, 2.0f});
  Viewer& AddEventData(const std::vector<EventArrayPtr>::const_iterator&,
                      const std::vector<EventArrayPtr>::const_iterator&, float, const std::string&,
                      const std::pair<float, float>& = {0.01f, 2.0f}, float = 1.0f);
  Viewer& AddEventData(const EventArrayPtr&, float, const std::string&,
                      const std::pair<float, float>& = {0.01f, 2.0f},
                      const std::optional<ns_viewer::Colour>& = {}, float = 1.0f);
  Viewer& AddEventData(const std::list<EventPtr>&, float, const std::string&,
                      const std::pair<float, float>& = {0.01f, 2.0f},
                      const std::optional<ns_viewer::Colour>& = {}, float = 1.0f);

  std::vector<std::size_t> AddEntity(const std::vector<ns_viewer::Entity::Ptr>&) override;
  std::vector<std::size_t> AddEntity(const std::vector<ns_viewer::Entity::Ptr>&,
                                    const std::string&) override;

 protected:
  std::map<std::string, std::vector<std::size_t>> _entities;
  CalibParamManagerPtr _parMagr;
  SplineBundleType::Ptr _splines;

  void ZoomInSplineCallBack();
  void ZoomOutSplineCallBack();
  void ZoomInCoordCallBack();
  void ZoomOutCoordCallBack();
  ns_viewer::MultiViewerConfigor GenViewerConfigor();
};

}  // namespace ns_ikalibr

#endif  // IKALIBR_VIEWER_PANGOLIN_H
