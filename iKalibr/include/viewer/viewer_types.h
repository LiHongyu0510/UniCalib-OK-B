// iKalibr viewer types: stub implementation (no tiny-viewer dependency).
// Replaces tiny-viewer with minimal types for compilation; visualization is disabled
// or can be replaced by file export / PCL / OpenCV later.
#ifndef IKALIBR_VIEWER_TYPES_H
#define IKALIBR_VIEWER_TYPES_H

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>

namespace ns_viewer {

struct Colour {
  float r = 0.f, g = 0.f, b = 0.f, a = 1.f;
  Colour() = default;
  Colour(float r_, float g_, float b_, float a_ = 1.f) : r(r_), g(g_), b(b_), a(a_) {}
  static Colour Black() { return Colour(0.f, 0.f, 0.f, 1.f); }
  static Colour Blue() { return Colour(0.f, 0.f, 1.f, 1.f); }
  static Colour Green() { return Colour(0.f, 1.f, 0.f, 1.f); }
  static Colour Red() { return Colour(1.f, 0.f, 0.f, 1.f); }
  Colour WithAlpha(float a_) const { return Colour(r, g, b, a_); }
};

struct Entity {
  using Ptr = std::shared_ptr<Entity>;
  using EntityPtr = Ptr;
  virtual ~Entity() = default;
  static Colour GetUniqueColour() {
    static int i = 0;
    float t[] = {0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
    Colour c(t[i % 5], t[(i + 1) % 5], t[(i + 2) % 5], 1.0f);
    ++i;
    return c;
  }
};

struct Posef {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  Posef() : R(Eigen::Matrix3f::Identity()), t(Eigen::Vector3f::Zero()) {}
  Posef(const Eigen::Matrix3f& R_, const Eigen::Vector3f& t_) : R(R_), t(t_) {}
};

class MultiViewer {
 public:
  virtual ~MultiViewer() = default;
  // Stub: accept and ignore any entity additions
  template <typename... Args>
  std::vector<std::size_t> AddEntity(Args&&...) { return {}; }
  template <typename... Args>
  void RemoveEntity(Args&&...) {}
  template <typename... Args>
  void SetViewerPose(Args&&...) {}
};

template <typename PointT>
struct AlignedCloud {
  using Ptr = std::shared_ptr<AlignedCloud>;
  using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
  struct CloudWrapper {
    PointCloudPtr GetCloud() const { return nullptr; }
  };
  CloudWrapper GetCloud() const { return {}; }
  static Ptr Create(const PointCloudPtr&, const Eigen::Vector3f&, float) {
    return std::make_shared<AlignedCloud>();
  }
};

struct Line : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<Line>(); }
};

struct IMU : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<IMU>(); }
};

struct Radar : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<Radar>(); }
};

struct CubeCamera : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<CubeCamera>(); }
};

struct LiDAR : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<LiDAR>(); }
};

struct LivoxLiDAR : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<LivoxLiDAR>(); }
};

struct Camera : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<Camera>(); }
};

struct RGBD : Entity {
  template <typename... Args>
  static Entity::Ptr Create(Args&&...) { return std::make_shared<RGBD>(); }
};

}  // namespace ns_viewer

#endif
