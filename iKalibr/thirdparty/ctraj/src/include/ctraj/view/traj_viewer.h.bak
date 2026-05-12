#ifndef CTRAJ_TRAJ_VIEWER_H
#define CTRAJ_TRAJ_VIEWER_H
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/core/pose.hpp"
#include <memory>
#include <string>

namespace ns_ctraj {
struct Viewer {
public:
    using Ptr = std::shared_ptr<Viewer>;
    explicit Viewer(const std::string &saveDir = "", const std::string &winName = "") { (void)saveDir; (void)winName; }
    static Ptr Create(const std::string &saveDir = "", const std::string &winName = "") {
        return std::make_shared<Viewer>(saveDir, winName);
    }
    void ShowPoseSequence(const Eigen::aligned_vector<Posed> &posSeq, float size = 0.3f) { (void)posSeq; (void)size; }
    void RunInSingleThread() {}
    void RunInMultiThread() {}
    void WaitForActive() {}
    bool IsActive() const { return false; }
    template<typename T> void AddEntity(T &&t) { (void)t; }
};
}
#endif
