#include "unicalib/extrinsic/lidar_scan_pair.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

namespace ns_unicalib {

std::pair<std::size_t, std::size_t> time_aligned_lidar_scan_indices_for_manual(
    const std::vector<LiDARScan>& ref,
    const std::vector<LiDARScan>& tgt) {
    if (ref.empty() || tgt.empty())
        return {0, 0};
    double best_dt = 1e300;
    std::size_t best_i = 0, best_j = 0;
    bool any = false;
    for (std::size_t i = 0; i < ref.size(); ++i) {
        if (!ref[i].cloud || ref[i].cloud->empty())
            continue;
        for (std::size_t j = 0; j < tgt.size(); ++j) {
            if (!tgt[j].cloud || tgt[j].cloud->empty())
                continue;
            const double dt = std::fabs(ref[i].timestamp - tgt[j].timestamp);
            if (!any || dt < best_dt) {
                best_dt = dt;
                best_i = i;
                best_j = j;
                any = true;
            }
        }
    }
    if (!any)
        return {0, 0};
    return {best_i, best_j};
}

}  // namespace ns_unicalib
