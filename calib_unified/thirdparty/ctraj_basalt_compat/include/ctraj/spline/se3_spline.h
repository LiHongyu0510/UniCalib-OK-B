// ctraj-compat: forward to basalt/spline/se3_spline.h
#pragma once
#include <basalt/spline/se3_spline.h>

// basalt uses ns_basalt; provide ns_ctraj alias
namespace ns_ctraj {
    template<int _N, typename _Scalar = double>
    using Se3Spline = basalt::Se3Spline<_N, _Scalar>;
}
