// ctraj-compat: forward to basalt/spline/so3_spline.h
#pragma once
#include <basalt/spline/so3_spline.h>

namespace ns_ctraj {
    template<int _N, typename _Scalar = double>
    using So3Spline = basalt::So3Spline<_N, _Scalar>;
}
