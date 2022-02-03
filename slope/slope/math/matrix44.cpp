#include "slope/math/matrix44.hpp"
#include "slope/math/quat.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

mat44 mat44::rotation(const vec3& axis, float angle)
{
    SL_ASSERT(axis.isfinite());
    SL_ASSERT(std::isfinite(angle));
    SL_ASSERT(slope::equal(axis.length(), 1.f));

    float cos = std::cos(angle);
    float sin = std::sin(angle);
    vec3 tmp = axis * (1.f - cos);

    return {
        cos + axis.x * tmp.x, axis.y * tmp.x + axis.z * sin, axis.z * tmp.x - axis.y * sin, 0.f,
        axis.x * tmp.y - axis.z * sin, cos + axis.y * tmp.y, axis.z * tmp.y + axis.x * sin, 0.f,
        axis.x * tmp.z + axis.y * sin, axis.y * tmp.z - axis.x * sin, cos + axis.z * tmp.z, 0.f,
        0.f, 0.f, 0.f, 1.f};
}

mat44::mat44(const quat& q)
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
    _03 = 0.f;
    _10 = 2.f * (xy - zw);
    _11 = 1.f - 2.f * (xx + zz);
    _12 = 2.f * (yz + xw);
    _13 = 0.f;
    _20 = 2.f * (xz + yw);
    _21 = 2.f * (yz - xw);
    _22 = 1.f - 2.f * (xx + yy);
    _23 = 0.f;
    _30 = 0.f;
    _31 = 0.f;
    _32 = 0.f;
    _33 = 1.f;
}

mat44 mat44::transposed() const
{
    return {
        _00, _10, _20, _30,
        _01, _11, _21, _31,
        _02, _12, _22, _32,
        _03, _13, _23, _33};
}

mat44 mat44::inverted_orthonormal() const
{
    float det = determinant();

    if (slope::equal(det, 0.f)) {
        return {};
    }

    float rcp = 1.f / det;

    mat44 res;

    res._00 = (_11 * _22 - _12 * _21) * rcp;
    res._01 = (_02 * _21 - _01 * _22) * rcp;
    res._02 = (_01 * _12 - _02 * _11) * rcp;
    res._10 = (_12 * _20 - _10 * _22) * rcp;
    res._11 = (_00 * _22 - _02 * _20) * rcp;
    res._12 = (_02 * _10 - _00 * _12) * rcp;
    res._20 = (_10 * _21 - _11 * _20) * rcp;
    res._21 = (_01 * _20 - _00 * _21) * rcp;
    res._22 = (_00 * _11 - _01 * _10) * rcp;

    res._30 = -(_30 * res._00 + _31 * res._10 + _32 * res._20);
    res._31 = -(_30 * res._01 + _31 * res._11 + _32 * res._21);
    res._32 = -(_30 * res._02 + _31 * res._12 + _32 * res._22);

    res._03 = 0.f;
    res._13 = 0.f;
    res._23 = 0.f;
    res._33 = 1.f;

    return res;
}

mat44 mat44::inverted() const
{
    float c1 = _22 * _33;
    float c2 = _32 * _23;
    float c3 = _12 * _33;
    float c4 = _32 * _13;
    float c5 = _12 * _23;
    float c6 = _22 * _13;
    float c7 = _02 * _33;
    float c8 = _32 * _03;
    float c9 = _02 * _23;
    float c10 = _22 * _03;
    float c11 = _02 * _13;
    float c12 = _12 * _03;
    float c13 = _20 * _31;
    float c14 = _30 * _21;
    float c15 = _10 * _31;
    float c16 = _30 * _11;
    float c17 = _10 * _21;
    float c18 = _20 * _11;
    float c19 = _00 * _31;
    float c20 = _30 * _01;
    float c21 = _00 * _21;
    float c22 = _20 * _01;
    float c23 = _00 * _11;
    float c24 = _10 * _01;

    mat44 result(
        (c1 * _11 + c4 * _21 + c5 * _31) - (c2 * _11 + c3 * _21 + c6 * _31),
        (c2 * _01 + c7 * _21 + c10 * _31) - (c1 * _01 + c8 * _21 + c9 * _31),
        (c3 * _01 + c8 * _11 + c11 * _31) - (c4 * _01 + c7 * _11 + c12 * _31),
        (c6 * _01 + c9 * _11 + c12 * _21) - (c5 * _01 + c10 * _11 + c11 * _21),
        (c2 * _10 + c3 * _20 + c6 * _30) - (c1 * _10 + c4 * _20 + c5 * _30),
        (c1 * _00 + c8 * _20 + c9 * _30) - (c2 * _00 + c7 * _20 + c10 * _30),
        (c4 * _00 + c7 * _10 + c12 * _30) - (c3 * _00 + c8 * _10 + c11 * _30),
        (c5 * _00 + c10 * _10 + c11 * _20) - (c6 * _00 + c9 * _10 + c12 * _20),
        (c13 * _13 + c16 * _23 + c17 * _33) - (c14 * _13 + c15 * _23 + c18 * _33),
        (c14 * _03 + c19 * _23 + c22 * _33) - (c13 * _03 + c20 * _23 + c21 * _33),
        (c15 * _03 + c20 * _13 + c23 * _33) - (c16 * _03 + c19 * _13 + c24 * _33),
        (c18 * _03 + c21 * _13 + c24 * _23) - (c17 * _03 + c22 * _13 + c23 * _23),
        (c15 * _22 + c18 * _32 + c14 * _12) - (c17 * _32 + c13 * _12 + c16 * _22),
        (c21 * _32 + c13 * _02 + c20 * _22) - (c19 * _22 + c22 * _32 + c14 * _02),
        (c19 * _12 + c24 * _32 + c16 * _02) - (c23 * _32 + c15 * _02 + c20 * _12),
        (c23 * _22 + c17 * _02 + c22 * _12) - (c21 * _12 + c24 * _22 + c18 * _02)
    );

    float det = _00 * result._00 + _10 * result._01 + _20 * result._02 + _30 * result._03;
    if (slope::equal(det, 0.f)) {
        return {};
    }

    float multiplier = 1.f / det;
    result._00 *= multiplier;
    result._01 *= multiplier;
    result._02 *= multiplier;
    result._03 *= multiplier;
    result._10 *= multiplier;
    result._11 *= multiplier;
    result._12 *= multiplier;
    result._13 *= multiplier;
    result._20 *= multiplier;
    result._21 *= multiplier;
    result._22 *= multiplier;
    result._23 *= multiplier;
    result._30 *= multiplier;
    result._31 *= multiplier;
    result._32 *= multiplier;
    result._33 *= multiplier;

    return result;
}

bool mat44::equal(const mat44& rhs, float epsilon) const
{
    return slope::equal(_00, rhs._00, epsilon) &&
        slope::equal(_01, rhs._01, epsilon) &&
        slope::equal(_02, rhs._02, epsilon) &&
        slope::equal(_03, rhs._03, epsilon) &&
        slope::equal(_10, rhs._10, epsilon) &&
        slope::equal(_11, rhs._11, epsilon) &&
        slope::equal(_12, rhs._12, epsilon) &&
        slope::equal(_13, rhs._13, epsilon) &&
        slope::equal(_20, rhs._20, epsilon) &&
        slope::equal(_21, rhs._21, epsilon) &&
        slope::equal(_22, rhs._22, epsilon) &&
        slope::equal(_23, rhs._23, epsilon) &&
        slope::equal(_30, rhs._30, epsilon) &&
        slope::equal(_31, rhs._31, epsilon) &&
        slope::equal(_32, rhs._32, epsilon) &&
        slope::equal(_33, rhs._33, epsilon);
}

bool mat44::is_finite() const
{
    return std::isfinite(_00) && std::isfinite(_01) && std::isfinite(_02) && std::isfinite(_03) &&
           std::isfinite(_10) && std::isfinite(_11) && std::isfinite(_12) && std::isfinite(_13) &&
           std::isfinite(_20) && std::isfinite(_21) && std::isfinite(_22) && std::isfinite(_23) &&
           std::isfinite(_30) && std::isfinite(_31) && std::isfinite(_32) && std::isfinite(_33);
}

void mat44::normalize_rotation()
{
    *reinterpret_cast<vec3*>(rows + 2) =    apply_to_unit_axis(2).normalized();
    *reinterpret_cast<vec3*>(rows) =        apply_to_unit_axis(1).cross(apply_to_unit_axis(2));
    *reinterpret_cast<vec3*>(rows) =        apply_to_unit_axis(0).normalized();
    *reinterpret_cast<vec3*>(rows + 1) =    apply_to_unit_axis(2).cross(apply_to_unit_axis(0));
}

} // slope
