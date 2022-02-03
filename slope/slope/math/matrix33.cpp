#include "slope/math/matrix33.hpp"
#include "slope/math/quat.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

mat33::mat33(const quat& q)
{
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

    _00 = 1.f - 2.f * (yy + zz);
    _01 = 2.f * (xy + zw);
    _02 = 2.f * (xz - yw);
    _10 = 2.f * (xy - zw);
    _11 = 1.f - 2.f * (xx + zz);
    _12 = 2.f * (yz + xw);
    _20 = 2.f * (xz + yw);
    _21 = 2.f * (yz - xw);
    _22 = 1.f - 2.f * (xx + yy);
}

mat33 mat33::inverted() const
{
    mat33 result(
        _22 * _11 - _12 * _21,
        _02 * _21 - _22 * _01,
        _12 * _01 - _02 * _11,
        _12 * _20 - _22 * _10,
        _22 * _00 - _02 * _20,
        _02 * _10 - _12 * _00,
        _10 * _21 - _20 * _11,
        _20 * _01 - _00 * _21,
        _00 * _11 - _10 * _01);

    float det = determinant();
    if (slope::equal(det, 0.f)) {
        return {};
    }

    float factor = 1.f / det;
    result._00 *= factor;
    result._01 *= factor;
    result._02 *= factor;
    result._10 *= factor;
    result._11 *= factor;
    result._12 *= factor;
    result._20 *= factor;
    result._21 *= factor;
    result._22 *= factor;

    return result;
}

mat33 mat33::transposed(const mat33& value) const
{
    return {
        value._00, value._10, value._20,
        value._01, value._11, value._21,
        value._02, value._12, value._22};
}

bool mat33::equal(const mat33& rhs, float epsilon) const
{
    return slope::equal(_00, rhs._00, epsilon) &&
           slope::equal(_01, rhs._01, epsilon) &&
           slope::equal(_02, rhs._02, epsilon) &&
           slope::equal(_10, rhs._10, epsilon) &&
           slope::equal(_11, rhs._11, epsilon) &&
           slope::equal(_12, rhs._12, epsilon) &&
           slope::equal(_20, rhs._20, epsilon) &&
           slope::equal(_21, rhs._21, epsilon) &&
           slope::equal(_22, rhs._22, epsilon);
}

bool mat33::is_finite() const
{
    return std::isfinite(_00) && std::isfinite(_01) && std::isfinite(_02) &&
           std::isfinite(_10) && std::isfinite(_11) && std::isfinite(_12) &&
           std::isfinite(_20) && std::isfinite(_21) && std::isfinite(_22);
}

} // slope
