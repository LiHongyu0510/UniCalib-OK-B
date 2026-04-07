// Stub for opengv when UNICALIB_NO_OPENGV is defined.
// Provides minimal placeholder types so compilation succeeds.
// opengv-dependent features are disabled at runtime.
#ifndef IKALIBR_OPENGV_STUB_H
#define IKALIBR_OPENGV_STUB_H

#include <vector>
#include <memory>
#include <Eigen/Core>

namespace opengv {

using point_t      = Eigen::Vector3d;
using bearing_t    = Eigen::Vector3d;
using points_t     = std::vector<point_t, Eigen::aligned_allocator<point_t>>;
using bearingVectors_t = std::vector<bearing_t, Eigen::aligned_allocator<bearing_t>>;

namespace sac {

// Minimal SampleConsensusProblem stub
template <typename ModelT>
class SampleConsensusProblem {
public:
    using model_t = ModelT;
    virtual ~SampleConsensusProblem() = default;
    virtual int getSampleSize() const { return 0; }
    virtual bool computeModelCoefficients(const std::vector<int>&, ModelT&) const { return false; }
    virtual void getDistancesToModel(const ModelT&, std::vector<double>&) const {}
    virtual void selectWithinDistance(const ModelT&, double, std::vector<int>&) const {}
    virtual int countWithinDistance(const ModelT&, double) const { return 0; }
    virtual void optimizeModelCoefficients(const std::vector<int>&, const ModelT&, ModelT&) {}
};

template <typename ProblemT>
class Ransac {
public:
    using ProblemPtr = std::shared_ptr<ProblemT>;
    ProblemPtr sac_model_;
    int max_iterations_{100};
    double threshold_{1e-3};
    std::vector<int> inliers_;
    typename ProblemT::model_t model_coefficients_;
    bool computeModel(int /*verbosity*/ = 0) { return false; }
};

}  // namespace sac

namespace types {
using rotation_t    = Eigen::Matrix3d;
using translation_t = Eigen::Vector3d;
using transformation_t = Eigen::Matrix<double, 3, 4>;
}  // namespace types

}  // namespace opengv

// Stub for common opengv headers
namespace opengv {
namespace absolute_pose {
struct CentralAbsoluteAdapter {};
}
namespace sac_problems {
namespace absolute_pose {
struct AbsolutePoseSacProblem {};
}
}
}  // namespace opengv

#endif  // IKALIBR_OPENGV_STUB_H
