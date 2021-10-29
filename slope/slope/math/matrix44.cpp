#include "slope/math/matrix44.hpp"
#include "slope/math/quat.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

Mat44 Mat44::rotation(const Vec3& axis, float angle) {
    SL_ASSERT(axis.isfinite());
    SL_ASSERT(std::isfinite(angle));
    SL_ASSERT(slope::equal(axis.length(), 1.f));

    float cos = std::cos(angle);
    float sin = std::sin(angle);
    Vec3 tmp = axis * (1.f - cos);

    return {
        cos + axis.x * tmp.x,          axis.y * tmp.x + axis.z * sin, axis.z * tmp.x - axis.y * sin, 0.f,
        axis.x * tmp.y - axis.z * sin, cos + axis.y * tmp.y,          axis.z * tmp.y + axis.x * sin, 0.f,
        axis.x * tmp.z + axis.y * sin, axis.y * tmp.z - axis.x * sin, cos + axis.z * tmp.z,          0.f,
        0.f,                           0.f,                           0.f,                           1.f };
}

Mat44 Mat44::scale(const Vec3& scale) {
    SL_ASSERT(scale.isfinite());

    return {
        scale.x,    0.f,     0.f,   0.f,
        0.f,    scale.y,     0.f,   0.f,
        0.f,    0.f,    scale.z,    0.f,
        0.f,    0.f,    0.f,  1.f };
}

Mat44 Mat44::translation(const Vec3& translation) {
    SL_ASSERT(translation.isfinite());

    return {
        1.f,       0.f,       0.f,       0.f,
        0.f,       1.f,       0.f,       0.f,
        0.f,       0.f,       1.f,       0.f,
        translation.x,  translation.y,  translation.z,  1.f};
}

Mat44::Mat44(const Quat& q) {
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
    _14 = 0.f;
    _21 = 2.f * (xy - zw);
    _22 = 1.f - 2.f * (xx + zz);
    _23 = 2.f * (yz + xw);
    _24 = 0.f;
    _31 = 2.f * (xz + yw);
    _32 = 2.f * (yz - xw);
    _33 = 1.f - 2.f * (xx + yy);
    _34 = 0.f;
    _41 = 0.f;
    _42 = 0.f;
    _43 = 0.f;
    _44 = 1.f;
}

Mat44 Mat44::inverted() const {
    float c1 = _33 * _44;
    float c2 = _43 * _34;
    float c3 = _23 * _44;
    float c4 = _43 * _24;
    float c5 = _23 * _34;
    float c6 = _33 * _24;
    float c7 = _13 * _44;
    float c8 = _43 * _14;
    float c9 = _13 * _34;
    float c10 = _33 * _14;
    float c11 = _13 * _24;
    float c12 = _23 * _14;
    float c13 = _31 * _42;
    float c14 = _41 * _32;
    float c15 = _21 * _42;
    float c16 = _41 * _22;
    float c17 = _21 * _32;
    float c18 = _31 * _22;
    float c19 = _11 * _42;
    float c20 = _41 * _12;
    float c21 = _11 * _32;
    float c22 = _31 * _12;
    float c23 = _11 * _22;
    float c24 = _21 * _12;

    Mat44 result(
            (c1 * _22 + c4 * _32 + c5 * _42) - (c2 * _22 + c3 * _32 + c6 * _42),
            (c2 * _12 + c7 * _32 + c10 * _42) - (c1 * _12 + c8 * _32 + c9 * _42),
            (c3 * _12 + c8 * _22 + c11 * _42) - (c4 * _12 + c7 * _22 + c12 * _42),
            (c6 * _12 + c9 * _22 + c12 * _32) - (c5 * _12 + c10 * _22 + c11 * _32),
            (c2 * _21 + c3 * _31 + c6 * _41) - (c1 * _21 + c4 * _31 + c5 * _41),
            (c1 * _11 + c8 * _31 + c9 * _41) - (c2 * _11 + c7 * _31 + c10 * _41),
            (c4 * _11 + c7 * _21 + c12 * _41) - (c3 * _11 + c8 * _21 + c11 * _41),
            (c5 * _11 + c10 * _21 + c11 * _31) - (c6 * _11 + c9 * _21 + c12 * _31),
            (c13 * _24 + c16 * _34 + c17 * _44) - (c14 * _24 + c15 * _34 + c18 * _44),
            (c14 * _14 + c19 * _34 + c22 * _44) - (c13 * _14 + c20 * _34 + c21 * _44),
            (c15 * _14 + c20 * _24 + c23 * _44) - (c16 * _14 + c19 * _24 + c24 * _44),
            (c18 * _14 + c21 * _24 + c24 * _34) - (c17 * _14 + c22 * _24 + c23 * _34),
            (c15 * _33 + c18 * _43 + c14 * _23) - (c17 * _43 + c13 * _23 + c16 * _33),
            (c21 * _43 + c13 * _13 + c20 * _33) - (c19 * _33 + c22 * _43 + c14 * _13),
            (c19 * _23 + c24 * _43 + c16 * _13) - (c23 * _43 + c15 * _13 + c20 * _23),
            (c23 * _33 + c17 * _13 + c22 * _23) - (c21 * _23 + c24 * _33 + c18 * _13)
    );

    float det = _11 * result._11 + _21 * result._12 + _31 * result._13 + _41 * result._14;
    if (slope::equal(det, 0.f)) {
        return {};
    }

    float multiplier = 1.f / det;
    result._11 *= multiplier;
    result._12 *= multiplier;
    result._13 *= multiplier;
    result._14 *= multiplier;
    result._21 *= multiplier;
    result._22 *= multiplier;
    result._23 *= multiplier;
    result._24 *= multiplier;
    result._31 *= multiplier;
    result._32 *= multiplier;
    result._33 *= multiplier;
    result._34 *= multiplier;
    result._41 *= multiplier;
    result._42 *= multiplier;
    result._43 *= multiplier;
    result._44 *= multiplier;

    return result;
}

} // slope