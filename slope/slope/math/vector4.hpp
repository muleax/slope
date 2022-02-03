#pragma once
#include "slope/math/math_utils.hpp"
#include "slope/math/vector3.hpp"

namespace slope {

class quat;

class vec4 {
public:
    static constexpr vec4   zero() { return {}; }

    constexpr               vec4() : x(0.f), y(0.f), z(0.f), w(0.f) {}
    explicit constexpr      vec4(float all) : x(all), y(all), z(all), w(all) {}
    constexpr               vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    explicit constexpr      vec4(const vec3& v) : x(v.x), y(v.y), z(v.z), w(1.f) {}
    constexpr               vec4(const vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}

    constexpr vec4          operator+(const vec4& rhs) const;
    constexpr vec4          operator-(const vec4& rhs) const;
    constexpr vec4&         operator+=(const vec4& value);
    constexpr vec4&         operator-=(const vec4& value);

    friend constexpr vec4   operator*(float lhs, const vec4& rhs);
    constexpr vec4          operator*(float rhs) const { return rhs * *this; }
    constexpr vec4          operator/(float rhs) const { return { x / rhs, y / rhs, z / rhs, w / rhs }; }
    constexpr vec4&         operator*=(float value);
    constexpr vec4&         operator/=(float value);

    constexpr vec4          operator-() const { return { -x, -y, -z, -w }; }

    constexpr bool          operator==(const vec4& value) const;
    constexpr bool          operator!=(const vec4& value) const;

    constexpr float&        operator[](size_t index) { return data[index]; }
    constexpr const float&  operator[](size_t index) const { return data[index]; }

    void                    set_zero();

    constexpr float         dot(const vec4& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w; }
    constexpr vec4          cross(const vec4& rhs) const;
    constexpr float         length_squared() const { return sqr(x) + sqr(y) + sqr(z) + sqr(w); }
    float                   length() const { return std::sqrt(length_squared()); }
    constexpr float         square_distance(const vec4& rhs) const;
    float                   distance(const vec4& rhs) const { return std::sqrt(square_distance(rhs)); }
    vec4                    normalized() const;

    bool                    is_zero() const { return x == 0.f && y == 0.f && z == 0.f && w == 0.f; }
    bool                    is_finite() const;
    constexpr bool          equal(const vec4& rhs, float epsilon = EQUALITY_EPSILON) const;
    constexpr bool          equal(float rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float x, y, z, w;
        };

        float data[3]{};
    };
};

constexpr vec4 lerp(const vec4& from, const vec4& to, float factor)
{
    return from + (to - from) * factor;
}

inline vec4 min(const vec4& lhs, const vec4& rhs)
{
    return {std::fmin(lhs.x, rhs.x), std::fmin(lhs.y, rhs.y), std::fmin(lhs.z, rhs.z), std::fmin(lhs.w, rhs.w)};
}

inline vec4 max(const vec4& lhs, const vec4& rhs)
{
    return {std::fmax(lhs.x, rhs.x), std::fmax(lhs.y, rhs.y), std::fmax(lhs.z, rhs.z), std::fmax(lhs.w, rhs.w)};
}

inline vec4 vec4::normalized() const
{
    float multiplier = 1.f / length();
    return {x * multiplier, y * multiplier, z * multiplier, w * multiplier};
}

constexpr vec4 operator*(float lhs, const vec4& rhs)
{
    return { rhs.x * lhs, rhs.y * lhs, rhs.z * lhs, rhs.w * lhs };
}

constexpr vec4 vec4::operator+(const vec4& rhs) const
{
    return {x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
}

constexpr vec4 vec4::operator-(const vec4& rhs) const
{
    return {x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w};
}

constexpr vec4& vec4::operator+=(const vec4& value)
{
    x += value.x;
    y += value.y;
    z += value.z;
    w += value.w;
    return *this;
}

constexpr vec4& vec4::operator-=(const vec4& value)
{
    x -= value.x;
    y -= value.y;
    z -= value.z;
    w -= value.w;
    return *this;
}

constexpr vec4& vec4::operator*=(float value)
{
    x *= value;
    y *= value;
    z *= value;
    w *= value;
    return *this;
}

constexpr vec4& vec4::operator/=(float value)
{
    float rcp = 1.f / value;
    x *= rcp;
    y *= rcp;
    z *= rcp;
    w *= rcp;
    return *this;
}

constexpr bool vec4::operator==(const vec4& value) const
{
    return x == value.x && y == value.y && z == value.z && w == value.w;
}

constexpr bool vec4::operator!=(const vec4& value) const
{
    return x != value.x || y != value.y || z != value.z || w != value.w;
}

inline void vec4::set_zero()
{
    x = 0.f;
    y = 0.f;
    z = 0.f;
    w = 0.f;
}

constexpr vec4 vec4::cross(const vec4& rhs) const
{
    return {y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x,
            1.f };
}

constexpr float vec4::square_distance(const vec4& rhs) const
{
    return sqr(x - rhs.x) + sqr(y - rhs.y) + sqr(z - rhs.z) + sqr(w - rhs.w);
}

inline bool vec4::is_finite() const
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
}

constexpr bool vec4::equal(const vec4& rhs, float epsilon) const
{
    return slope::equal(x, rhs.x, epsilon) &&
           slope::equal(y, rhs.y, epsilon) &&
           slope::equal(z, rhs.z, epsilon) &&
           slope::equal(w, rhs.w, epsilon);
}

constexpr bool vec4::equal(float rhs, float epsilon) const
{
    return slope::equal(x, rhs, epsilon) &&
           slope::equal(y, rhs, epsilon) &&
           slope::equal(z, rhs, epsilon) &&
           slope::equal(w, rhs, epsilon);
}

} // slope
