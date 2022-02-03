#pragma once
#include "slope/math/matrix22.hpp"
#include "slope/math/vector3.hpp"

namespace slope {

class quat;

class mat33 {
public:
    static mat33            cross(const vec3& v);

    constexpr               mat33();
    constexpr               mat33(  float _11, float _12, float _13,
                                    float _21, float _22, float _23,
                                    float _31, float _32, float _33);
    explicit constexpr      mat33(const mat22& value);
    constexpr               mat33(const vec3& r0, const vec3& r1, const vec3& r2) : rows{r0, r1, r2 } {}
    explicit                mat33(const quat& q);

    constexpr mat33         operator*(const mat33& rhs) const;
    constexpr mat33&        operator*=(const mat33& value) { return *this = *this * value; }

    constexpr mat33         operator+(const mat33& rhs) const;
    constexpr mat33         operator-(const mat33& rhs) const;
    constexpr mat33&        operator+=(const mat33& value) { return *this = *this + value; }
    constexpr mat33&        operator-=(const mat33& value) { return *this = *this - value; }

    friend constexpr mat33  operator*(float lhs, const mat33& rhs);
    constexpr mat33         operator*(float rhs) const;
    constexpr mat33         operator/(float rhs) const { return *this * (1.f / rhs); }
    constexpr mat33&        operator*=(float value) { return *this = *this * value; }
    constexpr mat33&        operator/=(float value) { return *this = *this / value; }

    friend constexpr vec3   operator*(const vec3& lhs, const mat33& rhs);
    constexpr vec3          operator*(const vec3& rhs) const;

    constexpr vec3&         operator[](size_t index) { return rows[index]; }
    constexpr const vec3&   operator[](size_t index) const { return rows[index]; }

    constexpr bool          operator==(const mat33& value) const;
    constexpr bool          operator!=(const mat33& value) const { return !(*this == value); }

    constexpr float         determinant() const { return _00 * _00 + _10 * _01 + _20 * _02; }

    mat33                   transposed(const mat33& value) const;
    mat33                   inverted() const;

    bool                    is_finite() const;
    bool                    equal(const mat33& rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float _00, _01, _02;
            float _10, _11, _12;
            float _20, _21, _22;
        };

        vec3 rows[3];
        float data[9];
    };
};

inline mat33 mat33::cross(const vec3& v)
{
    return {0.f, v.z, -v.y,
            -v.z, 0.f, v.x,
            v.y, -v.x, 0.f};
}

constexpr mat33::mat33()
    : _00(1.f), _01(0.f), _02(0.f)
    , _10(0.f), _11(1.f), _12(0.f)
    , _20(0.f), _21(0.f), _22(1.f) {}

constexpr mat33::mat33( float _11, float _12, float _13,
                        float _21, float _22, float _23,
                        float _31, float _32, float _33)
    : _00(_11), _01(_12), _02(_13)
    , _10(_21), _11(_22), _12(_23)
    , _20(_31), _21(_32), _22(_33) {}

constexpr mat33::mat33(const mat22& value)
    : _00(value._00), _01(value._01), _02(0.f)
    , _10(value._10), _11(value._11), _12(0.f)
    , _20(0.f), _21(0.f), _22(1.f) {}

constexpr mat33 mat33::operator+(const mat33& rhs) const {
    return {
        _00 + rhs._00, _01 + rhs._01, _02 + rhs._02,
        _10 + rhs._10, _11 + rhs._11, _12 + rhs._12,
        _20 + rhs._20, _21 + rhs._21, _22 + rhs._22 };
}

constexpr mat33 mat33::operator-(const mat33& rhs) const {
    return {
        _00 - rhs._00, _01 - rhs._01, _02 - rhs._02,
        _10 - rhs._10, _11 - rhs._11, _12 - rhs._12,
        _20 - rhs._20, _21 - rhs._21, _22 - rhs._22 };
}

constexpr mat33 mat33::operator*(const mat33& rhs) const
{
    return {
        _00 * rhs._00 + _01 * rhs._10 + _02 * rhs._20,
        _00 * rhs._01 + _01 * rhs._11 + _02 * rhs._21,
        _00 * rhs._02 + _01 * rhs._12 + _02 * rhs._22,
        _10 * rhs._00 + _11 * rhs._10 + _12 * rhs._20,
        _10 * rhs._01 + _11 * rhs._11 + _12 * rhs._21,
        _10 * rhs._02 + _11 * rhs._12 + _12 * rhs._22,
        _20 * rhs._00 + _21 * rhs._10 + _22 * rhs._20,
        _20 * rhs._01 + _21 * rhs._11 + _22 * rhs._21,
        _20 * rhs._02 + _21 * rhs._12 + _22 * rhs._22};
}

constexpr mat33 operator*(float lhs, const mat33& rhs)
{
    return rhs * lhs;
}

constexpr mat33 mat33::operator*(float rhs) const
{
    return {
        _00 * rhs, _01 * rhs, _02 * rhs,
        _10 * rhs, _11 * rhs, _12 * rhs,
        _20 * rhs, _21 * rhs, _22 * rhs};
}

constexpr vec3 mat33::operator*(const vec3& rhs) const
{
    return {
        _00 * rhs.x + _01 * rhs.y + _02 * rhs.z,
        _10 * rhs.x + _11 * rhs.y + _12 * rhs.z,
        _20 * rhs.x + _21 * rhs.y + _22 * rhs.z};
}

constexpr vec3 operator*(const vec3& lhs, const mat33& rhs)
{
    return rhs * lhs;
}

constexpr bool mat33::operator==(const mat33& value) const
{
    return _00 == value._00 && _01 == value._01 && _02 == value._02 &&
           _10 == value._10 && _11 == value._11 && _12 == value._12 &&
           _20 == value._20 && _21 == value._21 && _22 == value._22;
}

} // slope
