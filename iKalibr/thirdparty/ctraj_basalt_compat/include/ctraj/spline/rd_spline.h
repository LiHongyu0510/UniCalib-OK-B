// ctraj-compat: forward to basalt/spline/rd_spline.h
#pragma once
#include <basalt/spline/rd_spline.h>

namespace ns_ctraj {
    template<int _Dim, int _N, typename _Scalar = double>
    using RdSpline = basalt::RdSpline<_Dim, _N, _Scalar>;
}
