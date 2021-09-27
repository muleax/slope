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

constexpr Mat44 Mat44::inverted() const {
    float _1 = _33 * _44;
    float _2 = _43 * _34;
    float _3 = _23 * _44;
    float _4 = _43 * _24;
    float _5 = _23 * _34;
    float _6 = _33 * _24;
    float _7 = _13 * _44;
    float _8 = _43 * _14;
    float _9 = _13 * _34;
    float _10 = _33 * _14;
    float _11 = _13 * _24;
    float _12 = _23 * _14;
    float _13 = _31 * _42;
    float _14 = _41 * _32;
    float _15 = _21 * _42;
    float _16 = _41 * _22;
    float _17 = _21 * _32;
    float _18 = _31 * _22;
    float _19 = _11 * _42;
    float _20 = _41 * _12;
    float _21 = _11 * _32;
    float _22 = _31 * _12;
    float _23 = _11 * _22;
    float _24 = _21 * _12;

    Mat44 result(
            (_1 * _22 + _4 * _32 + _5 * _42) - (_2 * _22 + _3 * _32 + _6 * _42),
            (_2 * _12 + _7 * _32 + _10 * _42) - (_1 * _12 + _8 * _32 + _9 * _42),
            (_3 * _12 + _8 * _22 + _11 * _42) - (_4 * _12 + _7 * _22 + _12 * _42),
            (_6 * _12 + _9 * _22 + _12 * _32) - (_5 * _12 + _10 * _22 + _11 * _32),
            (_2 * _21 + _3 * _31 + _6 * _41) - (_1 * _21 + _4 * _31 + _5 * _41),
            (_1 * _11 + _8 * _31 + _9 * _41) - (_2 * _11 + _7 * _31 + _10 * _41),
            (_4 * _11 + _7 * _21 + _12 * _41) - (_3 * _11 + _8 * _21 + _11 * _41),
            (_5 * _11 + _10 * _21 + _11 * _31) - (_6 * _11 + _9 * _21 + _12 * _31),
            (_13 * _24 + _16 * _34 + _17 * _44) - (_14 * _24 + _15 * _34 + _18 * _44),
            (_14 * _14 + _19 * _34 + _22 * _44) - (_13 * _14 + _20 * _34 + _21 * _44),
            (_15 * _14 + _20 * _24 + _23 * _44) - (_16 * _14 + _19 * _24 + _24 * _44),
            (_18 * _14 + _21 * _24 + _24 * _34) - (_17 * _14 + _22 * _24 + _23 * _34),
            (_15 * _33 + _18 * _43 + _14 * _23) - (_17 * _43 + _13 * _23 + _16 * _33),
            (_21 * _43 + _13 * _13 + _20 * _33) - (_19 * _33 + _22 * _43 + _14 * _13),
            (_19 * _23 + _24 * _43 + _16 * _13) - (_23 * _43 + _15 * _13 + _20 * _23),
            (_23 * _33 + _17 * _13 + _22 * _23) - (_21 * _23 + _24 * _33 + _18 * _13)
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