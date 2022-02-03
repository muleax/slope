#pragma once
#include "slope/math/math_utils.hpp"

namespace slope {

class vec3 {
public:
    static constexpr vec3   zero() { return {}; }

    constexpr               vec3() : x(0.f), y(0.f), z(0.f) {}
    explicit constexpr      vec3(float all) : x(all), y(all), z(all) {}
    constexpr               vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    constexpr vec3          operator+(const vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z }; }
    constexpr vec3          operator-(const vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z }; }
    constexpr vec3&         operator+=(const vec3& value);
    constexpr vec3&         operator-=(const vec3& value);

    friend constexpr vec3   operator*(float lhs, const vec3& rhs) { return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z }; }
    constexpr vec3          operator*(float rhs) const { return rhs * *this; }
    constexpr vec3          operator/(float rhs) const { return {x / rhs, y / rhs, z / rhs }; }
    constexpr vec3&         operator*=(float value);
    constexpr vec3&         operator/=(float value);

    constexpr vec3          operator-() const { return {-x, -y, -z }; }

    constexpr bool          operator==(const vec3& value) const { return x == value.x && y == value.y && z == value.z; }
    constexpr bool          operator!=(const vec3& value) const { return x != value.x || y != value.y || z != value.z; }

    constexpr float&        operator[](size_t index) { return data[index]; }
    constexpr const float&  operator[](size_t index) const { return data[index]; }

    void                    set_zero();

    constexpr float         dot(const vec3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
    constexpr vec3          cross(const vec3& rhs) const;
    constexpr float         length_squared() const { return sqr(x) + sqr(y) + sqr(z); }
    float                   length() const { return std::sqrt(length_squared()); }
    constexpr float         square_distance(const vec3& rhs) const;
    float                   distance(const vec3& rhs) const { return std::sqrt(square_distance(rhs)); }
    vec3                    normalized() const;

    bool                    is_zero() const { return x == 0.f && y == 0.f && z == 0.f; }
    bool                    is_finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
    constexpr bool          equal(const vec3& rhs, float epsilon = EQUALITY_EPSILON) const;
    constexpr bool          equal(float rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float x, y, z;
        };

        float data[3]{};
    };
};

constexpr vec3 lerp(const vec3& from, const vec3& to, float factor)
{
    return from + (to - from) * factor;
}

inline vec3 min(const vec3& lhs, const vec3& rhs)
{
    return {std::fmin(lhs.x, rhs.x), std::fmin(lhs.y, rhs.y), std::fmin(lhs.z, rhs.z)};
}

inline vec3 max(const vec3& lhs, const vec3& rhs)
{
    return {std::fmax(lhs.x, rhs.x), std::fmax(lhs.y, rhs.y), std::fmax(lhs.z, rhs.z)};
}

inline vec3 vec3::normalized() const
{
    float multiplier = 1.f / length();
    return {x * multiplier, y * multiplier, z * multiplier};
}

constexpr vec3& vec3::operator+=(const vec3& value)
{
    x += value.x;
    y += value.y;
    z += value.z;
    return *this;
}

constexpr vec3& vec3::operator-=(const vec3& value)
{
    x -= value.x;
    y -= value.y;
    z -= value.z;
    return *this;
}

constexpr vec3& vec3::operator*=(float value)
{
    x *= value;
    y *= value;
    z *= value;
    return *this;
}

constexpr vec3& vec3::operator/=(float value)
{
    float rcp = 1.f / value;
    x *= rcp;
    y *= rcp;
    z *= rcp;
    return *this;
}

inline void vec3::set_zero()
{
    x = 0.f;
    y = 0.f;
    z = 0.f;
}

constexpr vec3 vec3::cross(const vec3& rhs) const
{
    return {y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x};
}

constexpr float vec3::square_distance(const vec3& rhs) const
{
    return sqr(x - rhs.x) + sqr(y - rhs.y) + sqr(z - rhs.z);
}

constexpr bool vec3::equal(const vec3& rhs, float epsilon) const
{
    return slope::equal(x, rhs.x, epsilon) && slope::equal(y, rhs.y, epsilon) && slope::equal(z, rhs.z, epsilon);
}

constexpr bool vec3::equal(float rhs, float epsilon) const
{
    return slope::equal(x, rhs, epsilon) && slope::equal(y, rhs, epsilon) && slope::equal(z, rhs, epsilon);
}

} // slope
