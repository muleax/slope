#pragma once
#include "slope/math/math_utils.hpp"

namespace slope {

class vec2 {
public:
    static constexpr vec2   zero() { return {}; }

    constexpr               vec2() : x(0.f), y(0.f) {}
    explicit constexpr      vec2(float all) : x(all), y(all) {}
    constexpr               vec2(float x, float y) : x(x), y(y) {}

    constexpr vec2          operator+(const vec2& rhs) const { return {x + rhs.x, y + rhs.y }; }
    constexpr vec2          operator-(const vec2& rhs) const { return {x - rhs.x, y - rhs.y }; }
    constexpr vec2&         operator+=(const vec2& value);
    constexpr vec2&         operator-=(const vec2& value);

    friend constexpr vec2   operator*(float lhs, const vec2& rhs) { return { lhs * rhs.x, lhs * rhs.y }; }
    constexpr vec2          operator*(float rhs) const { return rhs * *this; }
    constexpr vec2          operator/(float rhs) const { return {x / rhs, y / rhs }; }
    constexpr vec2&         operator*=(float value);
    constexpr vec2&         operator/=(float value);

    constexpr vec2          operator-() const { return {-x, -y }; }

    constexpr bool          operator==(const vec2& value) const { return x == value.x && y == value.y; }
    constexpr bool          operator!=(const vec2& value) const { return x != value.x || y != value.y; }

    constexpr float&        operator[](size_t index) { return data[index]; }
    constexpr const float&  operator[](size_t index) const { return data[index]; }

    void                    set_zero();

    constexpr float         dot(const vec2& rhs) const { return x * rhs.x + y * rhs.y; }
    constexpr float         length_squared() const { return sqr(x) + sqr(y); }
    float                   length() const { return std::sqrt(length_squared()); }
    constexpr float         square_distance(const vec2& rhs) const;
    float                   distance(const vec2& rhs) const;
    vec2                    normalized() const;

    bool                    is_zero() const { return x == 0.f && y == 0.f; }
    bool                    is_finite() const { return std::isfinite(x) && std::isfinite(y); }
    constexpr bool          equal(const vec2& rhs, float epsilon = EQUALITY_EPSILON) const;
    constexpr bool          equal(float rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float x, y;
        };

        float data[2]{};
    };
};

constexpr vec2 lerp(const vec2& from, const vec2& to, float factor)
{
    return from + (to - from) * factor;
}

inline vec2 min(const vec2& lhs, const vec2& rhs)
{
    return { std::fmin(lhs.x, rhs.x), std::fmin(lhs.y, rhs.y) };
}

inline vec2 max(const vec2& lhs, const vec2& rhs)
{
    return { std::fmax(lhs.x, rhs.x), std::fmax(lhs.y, rhs.y) };
}

constexpr vec2& vec2::operator+=(const vec2& value)
{
    x += value.x;
    y += value.y;
    return *this;
}

constexpr vec2& vec2::operator-=(const vec2& value)
{
    x -= value.x;
    y -= value.y;
    return *this;
}

constexpr vec2& vec2::operator*=(float value)
{
    x *= value;
    y *= value;
    return *this;
}

constexpr vec2& vec2::operator/=(float value)
{
    float rcp = 1.f / value;
    x *= rcp;
    y *= rcp;
    return *this;
}

inline void vec2::set_zero()
{
    x = 0.f;
    y = 0.f;
}

constexpr bool vec2::equal(const vec2& rhs, float epsilon) const
{
    return slope::equal(x, rhs.x, epsilon) && slope::equal(y, rhs.y, epsilon);
}

constexpr bool vec2::equal(float rhs, float epsilon) const
{
    return slope::equal(x, rhs, epsilon) && slope::equal(y, rhs, epsilon);
}

inline vec2 vec2::normalized() const
{
    float multiplier = 1.f / length();
    return { x * multiplier, y * multiplier};
}

constexpr float vec2::square_distance(const vec2& rhs) const
{
    return sqr(x - rhs.x) + sqr(y - rhs.y);
}

inline float vec2::distance(const vec2& rhs) const
{
    return std::sqrt(square_distance(rhs));
}

} // slope
