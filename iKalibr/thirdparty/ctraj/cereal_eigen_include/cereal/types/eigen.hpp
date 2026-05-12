#pragma once
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <Eigen/Dense>

namespace cereal {
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void save(Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
    const int rows = static_cast<int>(m.rows());
    const int cols = static_cast<int>(m.cols());
    ar(CEREAL_NVP(rows), CEREAL_NVP(cols));
    std::vector<_Scalar> vec(m.data(), m.data() + rows * cols);
    ar(CEREAL_NVP(vec));
}
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
    int rows, cols;
    ar(CEREAL_NVP(rows), CEREAL_NVP(cols));
    std::vector<_Scalar> vec;
    ar(CEREAL_NVP(vec));
    m.resize(rows, cols);
    if (static_cast<std::size_t>(rows * cols) == vec.size())
        std::copy(vec.begin(), vec.end(), m.data());
}
}
