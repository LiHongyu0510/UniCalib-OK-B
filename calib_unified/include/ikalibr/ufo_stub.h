// Stub for ufomap types when UNICALIB_NO_UFOMAP is defined.
// Types mirror the real ufomap API so pts_association.cpp compiles;
// all operations are no-ops at runtime.
#ifndef IKALIBR_UFO_STUB_H
#define IKALIBR_UFO_STUB_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <type_traits>
#include <Eigen/Core>

// ---------------------------------------------------------------------------
// ufo::math::Vector3<T> — mirrors real ufomap's ufo/math/vector3.h
// Required so that: getNormal() → Vector3<double> with .x .y .z plain members
// and dot(Vector3<float>) works via converting constructor.
// ---------------------------------------------------------------------------
namespace ufo {
namespace math {

template <typename T>
struct Vector3 {
    T x{0}, y{0}, z{0};

    constexpr Vector3() noexcept = default;
    constexpr Vector3(T x_, T y_, T z_) noexcept : x(x_), y(y_), z(z_) {}
    constexpr Vector3(const Vector3&) noexcept = default;

    // Converting constructor: Vector3<float> → Vector3<double> etc.
    template <typename U, typename = std::enable_if_t<!std::is_same<T, U>::value>>
    constexpr Vector3(const Vector3<U>& o) noexcept : x(o.x), y(o.y), z(o.z) {}

    constexpr T dot(const Vector3& o) const noexcept {
        return x * o.x + y * o.y + z * o.z;
    }

    constexpr T norm() const noexcept {
        return static_cast<T>(std::sqrt(x * x + y * y + z * z));
    }

    Vector3 normalize() const {
        T n = norm();
        if (n < static_cast<T>(1e-12)) return {0, 0, 0};
        return {x / n, y / n, z / n};
    }

    constexpr Vector3 operator+(const Vector3& o) const noexcept { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vector3 operator-(const Vector3& o) const noexcept { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vector3 operator*(T s)              const noexcept { return {x*s,   y*s,   z*s};   }
    constexpr Vector3 operator/(T s)              const noexcept { return {x/s,   y/s,   z/s};   }
};

}  // namespace math
}  // namespace ufo

// ---------------------------------------------------------------------------
// ufo namespace stubs
// ---------------------------------------------------------------------------
namespace ufo {
namespace geometry {
struct Point {
    float x{0}, y{0}, z{0};
    Point() = default;
    Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};
}  // namespace geometry

namespace map {

// Point3 = math::Vector3<float>, matching real ufomap typedef
using Point3 = math::Vector3<float>;

struct PointCloud {
    std::vector<Point3> points;
    void resize(std::size_t n) { points.resize(n); }
    std::size_t size() const { return points.size(); }
    Point3& operator[](std::size_t i) { return points[i]; }
    const Point3& operator[](std::size_t i) const { return points[i]; }
    auto begin()       { return points.begin(); }
    auto end()         { return points.end(); }
    auto begin() const { return points.begin(); }
    auto end()   const { return points.end(); }
};

struct Node {
    int depth{0};
    Node() = default;
    bool operator<(const Node& o) const { return depth < o.depth; }
};

// Predicate stub — supports operator&& and yields empty range in query()
struct PredicateStub {
    PredicateStub operator&&(const PredicateStub&) const { return {}; }
};

namespace predicate {
inline PredicateStub HasSurfel()                          { return {}; }
inline PredicateStub DepthMin(double)                     { return {}; }
inline PredicateStub DepthMax(double)                     { return {}; }
inline PredicateStub NumSurfelPointsMin(int)              { return {}; }
inline PredicateStub SurfelPlanarityMin(double)           { return {}; }
inline PredicateStub Contains(const geometry::Point&)     { return {}; }
}  // namespace predicate

struct SurfelMap {
    // Surfel: return types mirror real ufomap (ufo::math::Vector3)
    struct Surfel {
        math::Vector3<float>  mean;
        math::Vector3<double> norm{0, 0, 1};
        float n{0};

        double getPlanarity()            const { return 0; }
        math::Vector3<double> getNormal() const { return norm; }
        math::Vector3<float>  getMean()   const { return mean; }
    };

    std::vector<Node> queryNodes_;
    using iterator       = std::vector<Node>::iterator;
    using const_iterator = std::vector<Node>::const_iterator;

    SurfelMap() = default;
    SurfelMap(double, std::uint8_t) {}   // (resolution, depth) ctor

    const std::vector<Node>& query(const PredicateStub&) const { return queryNodes_; }
    bool isLeaf(const Node&)    const { return true;  }
    bool contains(const Node&)  const { return false; }
    bool hasSurfel(const Node&) const { return false; }
    Surfel getSurfel(const Node&)   const { return {}; }
    Point3 getNodeCenter(const Node&) const { return {}; }
    Point3 getNodeMin(const Node&)    const { return {}; }
    Point3 getNodeMax(const Node&)    const { return {}; }
    void insertPoint(const Point3&) {}
    template <typename Iter>
    void insertSurfelPoint(Iter, Iter) {}
    void clear() { queryNodes_.clear(); }
};

}  // namespace map
}  // namespace ufo

#endif  // IKALIBR_UFO_STUB_H
