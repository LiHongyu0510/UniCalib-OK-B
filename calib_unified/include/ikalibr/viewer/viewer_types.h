// iKalibr viewer types: stub implementation (no tiny-viewer dependency).
// Replaces tiny-viewer with minimal types for compilation; visualization is disabled
// or can be replaced by file export / PCL / OpenCV later.
#ifndef IKALIBR_VIEWER_TYPES_H
#define IKALIBR_VIEWER_TYPES_H

#include <Eigen/Core>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
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
  virtual std::vector<std::size_t> AddEntity(const std::vector<Entity::Ptr>&) { return {}; }
  virtual std::vector<std::size_t> AddEntity(const std::vector<Entity::Ptr>&, const std::string&) { return {}; }
  virtual void RemoveEntity(const std::vector<std::size_t>&, const std::string&) {}
  virtual void RemoveEntity(std::size_t, const std::string&) {}
  virtual void RunInMultiThread() {}
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
  Eigen::Vector3f from_{}, to_{};
  Colour color_{};
  float width_ = 0.01f;
  static Entity::Ptr Create(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Colour& c) {
    auto p = std::make_shared<Line>(); p->from_ = a; p->to_ = b; p->color_ = c; return p;
  }
  static Entity::Ptr Create(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Colour& c, float w) {
    auto p = std::make_shared<Line>(); p->from_ = a; p->to_ = b; p->color_ = c; p->width_ = w; return p;
  }
  static Entity::Ptr Create(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float w, const Colour& c) {
    auto p = std::make_shared<Line>(); p->from_ = a; p->to_ = b; p->width_ = w; p->color_ = c; return p;
  }
};
struct Landmark : Entity {
  static Entity::Ptr Create(const Eigen::Vector3f&) { return std::make_shared<Landmark>(); }
  static Entity::Ptr Create(const Eigen::Vector3f&, float, const Colour&) {
    return std::make_shared<Landmark>();
  }
};
template <typename PointT>
struct Cloud {
  using Ptr = std::shared_ptr<Cloud>;
  static Ptr Create(const std::vector<Entity::Ptr>&, float) { return std::make_shared<Cloud>(); }
  template <typename PtrOrCloud>
  static Ptr Create(const PtrOrCloud&, float) { return std::make_shared<Cloud>(); }
  template <typename PtrOrCloud>
  static Ptr Create(const PtrOrCloud&, const Colour&, float) { return std::make_shared<Cloud>(); }
};
struct Cube : Entity {
  static Entity::Ptr Create(const Posef&, bool, float, float, float, const Colour&) {
    return std::make_shared<Cube>();
  }
};
struct CubeCamera : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) {
    return std::make_shared<CubeCamera>();
  }
};
struct IMU : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) { return std::make_shared<IMU>(); }
};
struct Radar : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) { return std::make_shared<Radar>(); }
};
struct LiDAR : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) { return std::make_shared<LiDAR>(); }
};
struct LivoxLiDAR : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) {
    return std::make_shared<LivoxLiDAR>();
  }
};
struct Camera : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) { return std::make_shared<Camera>(); }
};
struct RGBD : Entity {
  static Entity::Ptr Create(const Posef&, float, const Colour&) { return std::make_shared<RGBD>(); }
};

// Extra stubs for any remaining tiny-viewer references
struct Posed {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    Posed() : R(Eigen::Matrix3f::Identity()), t(Eigen::Vector3f::Zero()) {}
    Posed(const Eigen::Matrix3f& R_, const Eigen::Vector3f& t_) : R(R_), t(t_) {}

    // Accept any Eigen matrix pair (e.g. double → float via cast)
    template<typename Derived1, typename Derived2>
    Posed(const Eigen::MatrixBase<Derived1>& R_, const Eigen::MatrixBase<Derived2>& t_)
        : R(R_.template cast<float>()), t(t_.template cast<float>()) {}

    // cast<T>() → Posef (always returns float version)
    template<typename T = float>
    Posef cast() const { return Posef(R, t); }
};

struct MultiViewerConfigor {
    struct GridOpts { bool showGrid = true; bool showIdentityCoord = true; };
    struct WindowOpts { int width = 640; int height = 480; };
    struct CameraOpts {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3f initPos = Eigen::Vector3f::Zero();
        double far = 1000.0;
    };
    std::string title;
    std::map<std::string, GridOpts> grid;
    std::map<std::string, CameraOpts> camera;
    WindowOpts window;
    std::map<char, std::function<void()>> callBacks;
    std::string screenShotSaveDir;

    MultiViewerConfigor() = default;
    MultiViewerConfigor(const std::string& t, int, int, int, const Colour& = {}) : title(t) {}
    explicit MultiViewerConfigor(const std::vector<std::string>& viewNames, const std::string& t)
        : title(t) {
        for (const auto& v : viewNames) { grid[v] = {}; camera[v] = {}; }
    }
    MultiViewerConfigor(const std::initializer_list<std::string>& viewNames, const std::string& t)
        : title(t) {
        for (const auto& v : viewNames) { grid[v] = {}; camera[v] = {}; }
    }
    MultiViewerConfigor& WithScreenShotSaveDir(const std::string& dir) {
        screenShotSaveDir = dir;
        return *this;
    }
};

struct Surfel : Entity {
    static Entity::Ptr Create(const Eigen::Vector4f&, const Cube&, bool, const Colour&) {
        return std::make_shared<Surfel>();
    }
    static Entity::Ptr Create(const Eigen::Vector4f&, const Cube&, bool, bool, const Colour&) {
        return std::make_shared<Surfel>();
    }
};

struct Coordinate : Entity {
    static Entity::Ptr Create(const Posed&, float) {
        return std::make_shared<Coordinate>();
    }
    static Entity::Ptr Create(const Posef&, float) {
        return std::make_shared<Coordinate>();
    }
};

struct Arrow : Entity {
    static Entity::Ptr Create(const Eigen::Vector3f&, const Eigen::Vector3f&,
                              const Colour&, float = 0.1f) {
        return std::make_shared<Arrow>();
    }
};

struct Sphere : Entity {
    static Entity::Ptr Create(const Eigen::Vector3f&, float, const Colour&) {
        return std::make_shared<Sphere>();
    }
};

struct Plane : Entity {
    static Entity::Ptr Create(const Eigen::Vector4f&, float, const Colour&) {
        return std::make_shared<Plane>();
    }
};

}  // namespace ns_viewer

#endif  // IKALIBR_VIEWER_TYPES_H
