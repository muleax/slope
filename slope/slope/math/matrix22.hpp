#pragma once
#include "slope/math/vector2.hpp"

namespace slope {

class mat22 {
public:
    constexpr               mat22();
    constexpr               mat22(float _11, float _12, float _21, float _22);
    constexpr               mat22(const vec2& r0, const vec2& r1) : rows{ r0, r1 } {}

    constexpr mat22         operator*(const mat22& rhs) const;
    constexpr mat22&        operator*=(const mat22& value) { return *this = *this * value; }

    constexpr mat22         operator+(const mat22& rhs) const;
    constexpr mat22         operator-(const mat22& rhs) const;
    constexpr mat22&        operator+=(const mat22& value) { return *this = *this + value; }
    constexpr mat22&        operator-=(const mat22& value) { return *this = *this - value; }

    friend constexpr mat22  operator*(float lhs, const mat22& rhs);
    constexpr mat22         operator*(float rhs) const  { return rhs * *this; }
    constexpr mat22         operator/(float rhs) const { return *this * (1.f / rhs); }
    constexpr mat22&        operator*=(float value) { return *this = *this * value; }
    constexpr mat22&        operator/=(float value) { return *this = *this / value; }

    constexpr vec2          operator*(const vec2& rhs) const;
    friend constexpr vec2   operator*(const vec2& lhs, const mat22& rhs);

    constexpr bool          operator==(const mat22& value) const;
    constexpr bool          operator!=(const mat22& value) const { return !(*this == value); }

    constexpr vec2&         operator[](size_t index) { return rows[index]; }
    constexpr const vec2&   operator[](size_t index) const { return rows[index]; }

    constexpr float         determinant() const { return _00 * _11 - _01 * _10; }

    mat22                   transposed() const;
    mat22                   inverted(const mat22& value) const;

    bool                    is_finite() const;
    bool                    equal(const mat22& rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float _00, _01;
            float _10, _11;
        };

        vec2    rows[2];
        float   data[4];
    };
};

constexpr mat22::mat22()
    : _00(1.f), _01(0.f)
    , _10(0.f), _11(1.f) {}

constexpr mat22::mat22(float _11, float _12, float _21, float _22)
    : _00(_11), _01(_12)
    , _10(_21), _11(_22) {}

constexpr mat22 mat22::operator+(const mat22& rhs) const
{
    return {
        _00 + rhs._00, _01 + rhs._01,
        _10 + rhs._10, _11 + rhs._11};
}

constexpr mat22 mat22::operator-(const mat22& rhs) const
{
    return {
        _00 - rhs._00, _01 - rhs._01,
        _10 - rhs._10, _11 - rhs._11};
}

constexpr mat22 mat22::operator*(const mat22& rhs) const
{
    return {
        _00 * rhs._00 + _01 * rhs._10, _00 * rhs._01 + _01 * rhs._11,
        _10 * rhs._00 + _11 * rhs._10, _10 * rhs._01 + _11 * rhs._11};
}

constexpr mat22 operator*(float lhs, const mat22& rhs)
{
    return {
        rhs._00 * lhs, rhs._01 * lhs,
        rhs._10 * lhs, rhs._11 * lhs};
}

constexpr vec2 operator*(const vec2& lhs, const mat22& rhs)
{
    return {
        lhs.x * rhs._00 + lhs.y * rhs._10,
        lhs.x * rhs._01 + lhs.y * rhs._11};
}

constexpr vec2 mat22::operator*(const vec2& rhs) const
{
    return {
        _00 * rhs.x + _01 * rhs.y,
        _10 * rhs.x + _11 * rhs.y};
}

constexpr bool mat22::operator==(const mat22& value) const
{
    return _00 == value._00 && _01 == value._01 && _10 == value._10 && _11 == value._11;
}

} // slope
