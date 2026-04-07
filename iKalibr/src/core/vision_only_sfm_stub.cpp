// Stub implementation of VisionOnlySfM when IKALIBR_VIEWER_DISABLED is set.
// The real vision_only_sfm.cpp requires tiny-viewer (removed).
// This stub provides the minimum needed for the calibration pipeline to compile and link.
// SfM functionality returns empty/false results.

#ifdef IKALIBR_VIEWER_DISABLED

#include "core/vision_only_sfm.h"
#include "calib/calib_param_manager.h"
#include "spdlog/spdlog.h"

namespace {
bool IKALIBR_UNIQUE_NAME(_2_) = ns_ikalibr::_1_(__FILE__);
}

namespace ns_ikalibr {

// ---- SfMFeaturePairInfo ----

std::optional<Sophus::SE3d> SfMFeaturePairInfo::RecoverRelativePose(
    const ns_veta::PinholeIntrinsic::Ptr &) const {
    return std::nullopt;
}

// ---- VisionOnlySfM ----

VisionOnlySfM::VisionOnlySfM(std::string topic,
                               const std::vector<CameraFramePtr> &frames,
                               CalibParamManagerPtr parMagr,
                               const So3SplineType &so3Spline,
                               ViewerPtr viewer)
    : _topic(std::move(topic)), _frames(frames), _parMagr(std::move(parMagr)),
      _so3Spline(so3Spline), _lmLabeler(0), _viewer(std::move(viewer)) {}

VisionOnlySfM::Ptr VisionOnlySfM::Create(const std::string &topic,
                                          const std::vector<CameraFramePtr> &frames,
                                          const CalibParamManagerPtr &parMagr,
                                          const So3SplineType &so3Spline,
                                          const ViewerPtr &viewer) {
    return std::make_shared<VisionOnlySfM>(topic, frames, parMagr, so3Spline, viewer);
}

bool VisionOnlySfM::PreProcess() {
    spdlog::warn("[VisionOnlySfM stub] PreProcess() disabled (viewer removed). Returning false.");
    return false;
}

bool VisionOnlySfM::StructureFromMotion() {
    spdlog::warn("[VisionOnlySfM stub] StructureFromMotion() disabled. Returning false.");
    return false;
}

const std::map<IndexPair, SfMFeaturePairInfo> &VisionOnlySfM::GetMatchRes() const {
    return _matchRes;
}

std::set<IndexPair> VisionOnlySfM::FindCovisibility(double) {
    return {};
}

// ---- Protected ----

std::optional<ns_veta::Posed> VisionOnlySfM::ComputeCamRotations(const CameraFramePtr &) {
    return std::nullopt;
}

std::map<CameraFramePtr, std::pair<polygon_2d, polygon_2d>>
VisionOnlySfM::FindCovisibility(const CameraFramePtr &, const std::set<IndexPair> &, double) {
    return {};
}

std::optional<std::pair<polygon_2d, polygon_2d>>
VisionOnlySfM::IntersectionArea(const cv::Mat &, const cv::Mat &, const Sophus::SO3d &) {
    return std::nullopt;
}

IndexPair VisionOnlySfM::InitStructure() {
    return {ns_veta::UndefinedIndexT, ns_veta::UndefinedIndexT};
}

cv::Mat VisionOnlySfM::DrawMatchResult(const IndexPair &) {
    return {};
}

void VisionOnlySfM::IncrementalSfM(const IndexPair &) {}

void VisionOnlySfM::CreateViewCubes() {}

void VisionOnlySfM::DrawMatchesInViewer(const ns_viewer::Colour &,
                                         const std::set<IndexPair> &,
                                         const ns_viewer::Colour &) const {}

std::map<IndexPair, Eigen::Vector3d>
VisionOnlySfM::Triangulate(const SfMFeaturePairInfo &, const Eigen::Matrix3d &,
                             const Eigen::Vector3d &, double, double) {
    return {};
}

void VisionOnlySfM::InsertTriangulateLM(const SfMFeaturePairInfo &,
                                         const std::map<IndexPair, Eigen::Vector3d> &,
                                         const ns_veta::Posed &) {}

void VisionOnlySfM::BatchOptimization() {}

// ---- Static ----

polygon_2d VisionOnlySfM::BufferPolygon(const polygon_2d &polygon, double) {
    return polygon;
}

std::optional<polygon_2d> VisionOnlySfM::ProjPolygon(const cv::Mat &, const Sophus::SO3d &,
                                                       const ns_veta::PinholeIntrinsic::Ptr &) {
    return std::nullopt;
}

void VisionOnlySfM::DrawProjPolygon(cv::Mat &, const polygon_2d &, const cv::Scalar &,
                                     const cv::Scalar &) {}

VisionOnlySfM::FeaturePack VisionOnlySfM::FindInBorderOnes(const FeaturePack &input,
                                                             const polygon_2d &) {
    return input;
}

cv::Mat VisionOnlySfM::DrawInBorderFeatMatch(const cv::Mat &img, const polygon_2d &,
                                              const FeaturePack &, const FeaturePack &) {
    return img.clone();
}

std::pair<SfMFeatureVec, SfMFeatureVec>
VisionOnlySfM::MatchFeatures(const FeaturePack &, const FeaturePack &) {
    return {{}, {}};
}

std::vector<int> VisionOnlySfM::RejectOutliers(const SfMFeatureVec &, const SfMFeatureVec &,
                                                const ns_veta::PinholeIntrinsic::Ptr &) {
    return {};
}

bool VisionOnlySfM::IsGraphConnect(const std::vector<IndexPair> &) {
    return false;
}

} // namespace ns_ikalibr

#endif // IKALIBR_VIEWER_DISABLED
