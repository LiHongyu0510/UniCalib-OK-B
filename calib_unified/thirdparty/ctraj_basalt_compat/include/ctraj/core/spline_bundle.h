// ctraj-compat: SplineBundle + double-time wrappers over basalt (So3Spline, RdSpline)
#ifndef CTRAJ_SPLINE_BUNDLE_H
#define CTRAJ_SPLINE_BUNDLE_H

#include <basalt/spline/rd_spline.h>
#include <basalt/spline/so3_spline.h>
#include <basalt/utils/assert.h>
#include <ctraj/spline/spline_segment.h>
#include <sophus/so3.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <array>
#include <map>
#include <ostream>
#include <utility>

namespace ns_ctraj {

namespace detail {

static constexpr double S_TO_NS = 1e9;
static constexpr double NS_TO_S = 1e-9;

inline int64_t toNs(double t_sec) {
  return static_cast<int64_t>(t_sec * S_TO_NS);
}

inline double toSec(int64_t t_ns) {
  return static_cast<double>(t_ns) * NS_TO_S;
}

}  // namespace detail

enum class SplineType { RdSpline, So3Spline };

struct SplineInfo {
  std::string name;
  SplineType type;
  double st, et, dt;

  SplineInfo(std::string name, SplineType type, double st, double et,
             double dt)
      : name(std::move(name)), type(type), st(st), et(et), dt(dt) {}
};

// Wrapper over basalt::So3Spline to expose double-time API (ctraj-compatible)
template <int Order>
class So3SplineWrapper {
 public:
  static constexpr int N = Order;
  static constexpr int DEG = Order - 1;
  using SO3 = Sophus::SO3d;
  using Vec3 = Eigen::Vector3d;
  using JacobianStruct = typename basalt::So3Spline<Order, double>::JacobianStruct;

  explicit So3SplineWrapper(double time_interval = 0.0, double start_time = 0.0)
      : impl_(detail::toNs(time_interval), detail::toNs(start_time)) {}

  double MinTime() const { return detail::toSec(impl_.minTimeNs()); }
  double MaxTime() const { return detail::toSec(impl_.maxTimeNs()); }
  double GetTimeInterval() const { return detail::toSec(impl_.getTimeIntervalNs()); }

  std::pair<double, size_t> ComputeTIndex(double timestamp) const {
    int64_t t_ns = detail::toNs(timestamp);
    int64_t st_ns = t_ns - impl_.minTimeNs();
    int64_t dt_ns = impl_.getTimeIntervalNs();
    int64_t s = st_ns / dt_ns;
    double u = static_cast<double>(st_ns % dt_ns) / static_cast<double>(dt_ns);
    return {u, static_cast<size_t>(s)};
  }

  void KnotsPushBack(const SO3& knot) { impl_.knotsPushBack(knot); }
  const Eigen::aligned_deque<SO3>& GetKnots() const { return impl_.getKnots(); }
  Eigen::aligned_deque<SO3>& GetKnots() { return impl_.getKnots(); }

  SO3 Evaluate(double time, JacobianStruct* J = nullptr) const {
    return impl_.evaluate(detail::toNs(time), J);
  }

  Vec3 VelocityBody(double time) const {
    return impl_.velocityBody(detail::toNs(time));
  }

  basalt::So3Spline<Order, double>& impl() { return impl_; }
  const basalt::So3Spline<Order, double>& impl() const { return impl_; }

  template <class Archive>
  void save(Archive& ar) const {
    ar(impl_.minTimeNs(), impl_.getTimeIntervalNs(), impl_.getKnots());
  }
  template <class Archive>
  void load(Archive& ar) {
    int64_t start_ns, dt_ns;
    Eigen::aligned_deque<SO3> knots;
    ar(start_ns, dt_ns, knots);
    impl_ = basalt::So3Spline<Order, double>(dt_ns, start_ns);
    for (const auto& k : knots) impl_.knotsPushBack(k);
  }

 private:
  basalt::So3Spline<Order, double> impl_;
};

// Wrapper over basalt::RdSpline<3> to expose double-time API (ctraj-compatible)
template <int Order>
class RdSplineWrapper {
 public:
  static constexpr int N = Order;
  static constexpr int DEG = Order - 1;
  using VecD = Eigen::Vector3d;

  explicit RdSplineWrapper(double time_interval = 0.0, double start_time = 0.0)
      : impl_(detail::toNs(time_interval), detail::toNs(start_time)) {}

  double MinTime() const { return detail::toSec(impl_.minTimeNs()); }
  double MaxTime() const { return detail::toSec(impl_.maxTimeNs()); }
  double GetTimeInterval() const { return detail::toSec(impl_.getTimeIntervalNs()); }

  std::pair<double, size_t> ComputeTIndex(double timestamp) const {
    int64_t t_ns = detail::toNs(timestamp);
    int64_t st_ns = t_ns - impl_.minTimeNs();
    int64_t dt_ns = impl_.getTimeIntervalNs();
    int64_t s = st_ns / dt_ns;
    double u = static_cast<double>(st_ns % dt_ns) / static_cast<double>(dt_ns);
    return {u, static_cast<size_t>(s)};
  }

  void KnotsPushBack(const VecD& knot) { impl_.knotsPushBack(knot); }
  const Eigen::aligned_deque<VecD>& GetKnots() const { return impl_.getKnots(); }
  Eigen::aligned_deque<VecD>& GetKnots() { return impl_.getKnots(); }

  VecD Evaluate(double time) const {
    return impl_.evaluate<0>(detail::toNs(time));
  }

  basalt::RdSpline<3, Order, double>& impl() { return impl_; }
  const basalt::RdSpline<3, Order, double>& impl() const { return impl_; }

  template <class Archive>
  void save(Archive& ar) const {
    ar(impl_.minTimeNs(), impl_.getTimeIntervalNs(), impl_.getKnots());
  }
  template <class Archive>
  void load(Archive& ar) {
    int64_t start_ns, dt_ns;
    Eigen::aligned_deque<VecD> knots;
    ar(start_ns, dt_ns, knots);
    impl_ = basalt::RdSpline<3, Order, double>(dt_ns, start_ns);
    for (const auto& k : knots) impl_.knotsPushBack(k);
  }

 private:
  basalt::RdSpline<3, Order, double> impl_;
};

template <int Order>
class SplineBundle {
 public:
  static constexpr int N = Order;
  using Ptr = std::shared_ptr<SplineBundle>;
  using RdSplineType = RdSplineWrapper<Order>;
  using So3SplineType = So3SplineWrapper<Order>;
  using RdSplineKnotType = Eigen::Vector3d;
  using So3SplineKnotType = Sophus::SO3d;
  using SplineMetaType = ns_ctraj::SplineMeta<Order>;

 private:
  std::map<std::string, So3SplineType> _so3Splines;
  std::map<std::string, RdSplineType> _rdSplines;

  template <class WrapperType, class KnotType>
  static void ExtendKnotsTo(WrapperType& spline, double t_sec,
                            const KnotType& init) {
    while (spline.GetKnots().size() < static_cast<size_t>(N) ||
           spline.MaxTime() < t_sec) {
      spline.KnotsPushBack(init);
    }
  }

 public:
  explicit SplineBundle(const std::vector<SplineInfo>& splines) {
    for (const auto& spline : splines) {
      AddSpline(spline);
    }
  }

  SplineBundle& AddSpline(const SplineInfo& spline) {
    switch (spline.type) {
      case SplineType::RdSpline: {
        _rdSplines.insert(
            {spline.name, RdSplineType(spline.dt, spline.st)});
        ExtendKnotsTo(_rdSplines.at(spline.name), spline.et,
                      RdSplineKnotType::Zero());
      } break;
      case SplineType::So3Spline: {
        _so3Splines.insert(
            {spline.name, So3SplineType(spline.dt, spline.st)});
        ExtendKnotsTo(_so3Splines.at(spline.name), spline.et,
                      So3SplineKnotType());
      } break;
    }
    return *this;
  }

  static Ptr Create(const std::vector<SplineInfo>& splines) {
    return std::make_shared<SplineBundle>(splines);
  }

  void Save(const std::string& filename) const {
    std::ofstream file(filename);
    cereal::JSONOutputArchive ar(file);
    ar(cereal::make_nvp("SplineBundle", *this));
  }

  static SplineBundle::Ptr Load(const std::string& filename) {
    auto bundle = SplineBundle::Create({});
    std::ifstream file(filename);
    cereal::JSONInputArchive ar(file);
    ar(cereal::make_nvp("SplineBundle", *bundle));
    return bundle;
  }

  So3SplineType& GetSo3Spline(const std::string& name) {
    return _so3Splines.at(name);
  }

  RdSplineType& GetRdSpline(const std::string& name) {
    return _rdSplines.at(name);
  }

  bool TimeInRangeForSo3(double time, const std::string& name) const {
    const auto& spline = _so3Splines.at(name);
    return time >= spline.MinTime() + 1E-6 && time < spline.MaxTime() - 1E-6;
  }

  bool TimeInRangeForRd(double time, const std::string& name) const {
    const auto& spline = _rdSplines.at(name);
    return time >= spline.MinTime() + 1E-6 && time < spline.MaxTime() - 1E-6;
  }

  template <class SplineType>
  bool TimeInRange(double time, const SplineType& spline) const {
    return time >= spline.MinTime() + 1E-6 && time < spline.MaxTime() - 1E-6;
  }

  void CalculateSo3SplineMeta(const std::string& name, time_init_t times,
                             SplineMetaType& splineMeta) const {
    CalculateSplineMeta(_so3Splines.at(name), times, splineMeta);
  }

  void CalculateRdSplineMeta(const std::string& name, time_init_t times,
                             SplineMetaType& splineMeta) const {
    CalculateSplineMeta(_rdSplines.at(name), times, splineMeta);
  }

  template <class WrapperType>
  static void CalculateSplineMeta(const WrapperType& spline,
                                  time_init_t times,
                                  SplineMetaType& splineMeta) {
    double master_dt = spline.GetTimeInterval();
    double master_t0 = spline.MinTime();
    size_t current_segment_start = 0;
    size_t current_segment_end = 0;

    for (auto tt : times) {
      std::pair<double, size_t> ui_1 = spline.ComputeTIndex(tt.first);
      std::pair<double, size_t> ui_2 = spline.ComputeTIndex(tt.second);
      size_t i1 = ui_1.second;
      size_t i2 = ui_2.second;

      if (splineMeta.segments.empty() || i1 > current_segment_end) {
        double segment_t0 = master_t0 + master_dt * static_cast<double>(i1);
        splineMeta.segments.emplace_back(segment_t0, master_dt);
        current_segment_start = i1;
      } else {
        i1 = current_segment_end + 1;
      }

      auto& current_segment_meta = splineMeta.segments.back();
      for (size_t i = i1; i < (i2 + N); ++i) {
        current_segment_meta.n += 1;
      }
      current_segment_end = current_segment_start + current_segment_meta.n - 1;
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const SplineBundle& bundle) {
    os << "SplineBundle:\n";
    for (const auto& [name, spline] : bundle._so3Splines) {
      os << "'name': " << name << ", 'SplineType::So3Spline', ['st': "
         << spline.MinTime() << ", 'et': " << spline.MaxTime()
         << ", 'dt': " << spline.GetTimeInterval() << "]\n";
    }
    for (const auto& [name, spline] : bundle._rdSplines) {
      os << "'name': " << name << ", 'SplineType::RdSpline', ['st': "
         << spline.MinTime() << ", 'et': " << spline.MaxTime()
         << ", 'dt': " << spline.GetTimeInterval() << "]\n";
    }
    return os;
  }

  template <class Archive>
  void serialize(Archive& ar) {
    ar(cereal::make_nvp("So3Splines", _so3Splines),
       cereal::make_nvp("RdSplines", _rdSplines));
  }
};

}  // namespace ns_ctraj

#endif
