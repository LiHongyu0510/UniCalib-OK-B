#ifndef TINY_VIEWER_VIEWER_H
#define TINY_VIEWER_VIEWER_H

#include <Eigen/Dense>

namespace tiny_viewer {

struct Viewer {
    Viewer() = default;
    void Initialize() {}
    void Update() {}
    void Render() {}
    void Shutdown() {}
};

} // namespace tiny_viewer

#endif
