#pragma once
#include "slope/math/matrix33.hpp"
#include "slope/math/vector4.hpp"

namespace slope
{

class mat44 {
public:
    constexpr static mat44  identity() { return {}; }
    static mat44            rotation(const vec3& axis, float angle);
    constexpr static mat44  scale(const vec3& scale);
    constexpr static mat44  translate(const vec3& translation);

    constexpr               mat44();
    constexpr               mat44(  float _11, float _12, float _13, float _14,
                                    float _21, float _22, float _23, float _24,
                                    float _31, float _32, float _33, float _34,
                                    float _41, float _42, float _43, float _44);
    explicit constexpr      mat44(const mat22& value);
    explicit constexpr      mat44(const mat33& value);
    constexpr               mat44(const vec4& r0, const vec4& r1, const vec4& r2, const vec4& r3);
    explicit                mat44(const quat& q);

    constexpr mat44         operator*(const mat44& rhs) const;
    constexpr mat44&        operator*=(const mat44& value) { return *this = *this * value; }

    constexpr mat44         operator+(const mat44& rhs) const;
    constexpr mat44         operator-(const mat44& rhs) const;
    constexpr mat44&        operator+=(const mat44& value) { return *this = *this + value; }
    constexpr mat44&        operator-=(const mat44& value) { return *this = *this - value; }

    friend constexpr mat44  operator*(float lhs, const mat44& rhs);
    constexpr mat44         operator*(float rhs) const;
    constexpr mat44         operator/(float rhs) const { return *this * (1.f / rhs); }
    constexpr mat44&        operator*=(float value) { return *this = *this * value; }
    constexpr mat44&        operator/=(float value) { return *this = *this / value; }

    friend constexpr vec3   operator*(const vec3& lhs, const mat44& rhs);
    constexpr vec3          operator*(const vec3& rhs)  const;
    friend constexpr vec4   operator*(const vec4& lhs, const mat44& rhs);
    constexpr vec4          operator*(const vec4& rhs) const;

    constexpr bool          operator==(const mat44& value) const;
    constexpr bool          operator!=(const mat44& value) const { return !(*this == value); }

    constexpr vec4&         operator[](size_t index) { return rows[index]; }
    constexpr const vec4&   operator[](size_t index) const { return rows[index]; }

    void                    normalize_rotation();
    void                    set_translation(const vec3& translation);

    constexpr vec3          apply_point(const vec3& point) const;
    constexpr vec3          apply_normal(const vec3& normal) const;

    const vec3&             translation() const;
    const vec3&             apply_to_unit_axis(uint32_t axis) const;

    constexpr float         determinant() const;

    mat44                   transposed() const;
    mat44                   inverted_orthonormal() const;
    mat44                   inverted() const;

    bool                    is_finite() const;
    bool                    equal(const mat44& rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float _00, _01, _02, _03;
            float _10, _11, _12, _13;
            float _20, _21, _22, _23;
            float _30, _31, _32, _33;
        };

        vec4 rows[4];
        float data[16];
    };
};

constexpr mat44::mat44()
    : _00(1.f), _01(0.f), _02(0.f), _03(0.f)
    , _10(0.f), _11(1.f), _12(0.f), _13(0.f)
    , _20(0.f), _21(0.f), _22(1.f), _23(0.f)
    , _30(0.f), _31(0.f), _32(0.f), _33(1.f) {}

constexpr mat44::mat44( float _11, float _12, float _13, float _14,
                        float _21, float _22, float _23, float _24,
                        float _31, float _32, float _33, float _34,
                        float _41, float _42, float _43, float _44)
    : _00(_11), _01(_12), _02(_13), _03(_14)
    , _10(_21), _11(_22), _12(_23), _13(_24)
    , _20(_31), _21(_32), _22(_33), _23(_34)
    , _30(_41), _31(_42), _32(_43), _33(_44) {}

constexpr mat44::mat44(const mat22& value)
    : _00(value._00), _01(value._01), _02(0.f), _03(0.f)
    , _10(value._10), _11(value._11), _12(0.f), _13(0.f)
    , _20(0.f), _21(0.f), _22(1.f), _23(0.f)
    , _30(0.f), _31(0.f), _32(0.f), _33(1.f) {}

constexpr mat44::mat44(const mat33& value)
    : _00(value._00), _01(value._01), _02(value._02), _03(0.f)
    , _10(value._10), _11(value._11), _12(value._12), _13(0.f)
    , _20(value._20), _21(value._21), _22(value._22), _23(0.f)
    , _30(0.f), _31(0.f), _32(0.f), _33(1.f) {}

constexpr mat44::mat44(const vec4& r0, const vec4& r1, const vec4& r2, const vec4& r3)
    : rows{ r0, r1, r2, r3 } {}

constexpr mat44 mat44::operator+(const mat44& rhs) const
{
    return {
        _00 + rhs._00, _01 + rhs._01, _02 + rhs._02, _03 + rhs._03,
        _10 + rhs._10, _11 + rhs._11, _12 + rhs._12, _13 + rhs._13,
        _20 + rhs._20, _21 + rhs._21, _22 + rhs._22, _23 + rhs._23,
        _30 + rhs._30, _31 + rhs._31, _32 + rhs._32, _33 + rhs._33};
}

constexpr mat44 mat44::operator-(const mat44& rhs) const
{
    return {
        _00 - rhs._00, _01 - rhs._01, _02 - rhs._02, _03 - rhs._03,
        _10 - rhs._10, _11 - rhs._11, _12 - rhs._12, _13 - rhs._13,
        _20 - rhs._20, _21 - rhs._21, _22 - rhs._22, _23 - rhs._23,
        _30 - rhs._30, _31 - rhs._31, _32 - rhs._32, _33 - rhs._33};
}

constexpr mat44 mat44::operator*(const mat44& rhs) const
{
    return {
        _00 * rhs._00 + _01 * rhs._10 + _02 * rhs._20 + _03 * rhs._30,
        _00 * rhs._01 + _01 * rhs._11 + _02 * rhs._21 + _03 * rhs._31,
        _00 * rhs._02 + _01 * rhs._12 + _02 * rhs._22 + _03 * rhs._32,
        _00 * rhs._03 + _01 * rhs._13 + _02 * rhs._23 + _03 * rhs._33,
        _10 * rhs._00 + _11 * rhs._10 + _12 * rhs._20 + _13 * rhs._30,
        _10 * rhs._01 + _11 * rhs._11 + _12 * rhs._21 + _13 * rhs._31,
        _10 * rhs._02 + _11 * rhs._12 + _12 * rhs._22 + _13 * rhs._32,
        _10 * rhs._03 + _11 * rhs._13 + _12 * rhs._23 + _13 * rhs._33,
        _20 * rhs._00 + _21 * rhs._10 + _22 * rhs._20 + _23 * rhs._30,
        _20 * rhs._01 + _21 * rhs._11 + _22 * rhs._21 + _23 * rhs._31,
        _20 * rhs._02 + _21 * rhs._12 + _22 * rhs._22 + _23 * rhs._32,
        _20 * rhs._03 + _21 * rhs._13 + _22 * rhs._23 + _23 * rhs._33,
        _30 * rhs._00 + _31 * rhs._10 + _32 * rhs._20 + _33 * rhs._30,
        _30 * rhs._01 + _31 * rhs._11 + _32 * rhs._21 + _33 * rhs._31,
        _30 * rhs._02 + _31 * rhs._12 + _32 * rhs._22 + _33 * rhs._32,
        _30 * rhs._03 + _31 * rhs._13 + _32 * rhs._23 + _33 * rhs._33};
}

constexpr mat44 operator*(float lhs, const mat44& rhs)
{
    return rhs * lhs;
}

constexpr mat44 mat44::operator*(float rhs) const
{
    return {
        _00 * rhs, _01 * rhs, _02 * rhs, _03 * rhs,
        _10 * rhs, _11 * rhs, _12 * rhs, _13 * rhs,
        _20 * rhs, _21 * rhs, _22 * rhs, _23 * rhs,
        _30 * rhs, _31 * rhs, _32 * rhs, _33 * rhs};
}

constexpr vec3 operator*(const vec3& lhs, const mat44& rhs)
{
    return rhs * lhs;
}

constexpr vec3 mat44::operator*(const vec3& rhs)  const
{
    return {
        _00 * rhs.x + _01 * rhs.y + _02 * rhs.z,
        _10 * rhs.x + _11 * rhs.y + _12 * rhs.z,
        _20 * rhs.x + _21 * rhs.y + _22 * rhs.z};
}

constexpr vec4 operator*(const vec4& lhs, const mat44& rhs)
{
    return rhs * lhs;
}

constexpr vec4 mat44::operator*(const vec4& rhs) const
{
    return {
        _00 * rhs.x + _01 * rhs.y + _02 * rhs.z + _03 * rhs.w,
        _10 * rhs.x + _11 * rhs.y + _12 * rhs.z + _13 * rhs.w,
        _20 * rhs.x + _21 * rhs.y + _22 * rhs.z + _23 * rhs.w,
        _30 * rhs.x + _31 * rhs.y + _32 * rhs.z + _33 * rhs.w};
}

constexpr bool mat44::operator==(const mat44& value) const
{
    return _00 == value._00 && _01 == value._01 && _02 == value._02 && _03 == value._03 &&
           _10 == value._10 && _11 == value._11 && _12 == value._12 && _13 == value._13 &&
           _20 == value._20 && _21 == value._21 && _22 == value._22 && _23 == value._23 &&
           _30 == value._30 && _31 == value._31 && _32 == value._32 && _33 == value._33;
}

constexpr vec3 mat44::apply_point(const vec3& point) const
{
    return {
        point.x * _00 + point.y * _10 + point.z * _20 + _30,
        point.x * _01 + point.y * _11 + point.z * _21 + _31,
        point.x * _02 + point.y * _12 + point.z * _22 + _32};
}

constexpr vec3 mat44::apply_normal(const vec3& normal) const
{
    return {
        normal.x * _00 + normal.y * _10 + normal.z * _20,
        normal.x * _01 + normal.y * _11 + normal.z * _21,
        normal.x * _02 + normal.y * _12 + normal.z * _22};
}

inline const vec3& mat44::translation() const
{
    return *reinterpret_cast<const vec3*>(rows + 3);
}

inline const vec3& mat44::apply_to_unit_axis(uint32_t axis) const
{
    return *reinterpret_cast<const vec3*>(rows + axis);
}

inline void mat44::set_translation(const vec3& translation)
{
    rows[3].x = translation.x;
    rows[3].y = translation.y;
    rows[3].z = translation.z;
}

constexpr float mat44::determinant() const
{
    float det = _00 * (_11 * _22 - _12 * _21);
    det -= _01 * (_10 * _22 - _12 * _20);
    det += _02 * (_10 * _21 - _11 * _20);
    return det;
}

constexpr mat44 mat44::scale(const vec3& scale)
{
    SL_ASSERT(scale.isfinite());

    return {
        scale.x, 0.f, 0.f, 0.f,
        0.f, scale.y, 0.f, 0.f,
        0.f, 0.f, scale.z, 0.f,
        0.f, 0.f, 0.f, 1.f};
}

constexpr mat44 mat44::translate(const vec3& translation)
{
    SL_ASSERT(translation.isfinite());

    return {
        1.f, 0.f, 0.f, 0.f,
        0.f, 1.f, 0.f, 0.f,
        0.f, 0.f, 1.f, 0.f,
        translation.x, translation.y, translation.z, 1.f};
}

} // slope
