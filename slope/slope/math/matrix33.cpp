#include "slope/math/matrix33.hpp"
#include "slope/math/quat.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

Mat33::Mat33(const Quat& q) {
    SL_ASSERT(q.isfinite());

    float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zz = q.z * q.z;
    float zw = q.z * q.w;

    _11 = 1.f - 2.f * (yy + zz);
    _12 = 2.f * (xy + zw);
    _13 = 2.f * (xz - yw);
    _21 = 2.f * (xy - zw);
    _22 = 1.f - 2.f * (xx + zz);
    _23 = 2.f * (yz + xw);
    _31 = 2.f * (xz + yw);
    _32 = 2.f * (yz - xw);
    _33 = 1.f - 2.f * (xx + yy);
}

} // slope