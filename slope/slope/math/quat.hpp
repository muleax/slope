#pragma once
#include "slope/math/vector4.hpp"

namespace slope {

class mat33;
class mat44;

class quat {
public:
    static quat             rotation(const vec3& axis, float angle);
    static quat             shortest_arc(const vec3& from, const vec3& to, const vec3& spin_axis);

    constexpr               quat() : x(0.f), y(0.f), z(0.f), w(1.f) {}
    constexpr               quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    constexpr               quat(const vec3& value, float w) : x(value.x), y(value.y), z(value.z), w(w) {}
    explicit constexpr      quat(const vec4& value) : x(value.x), y(value.y), z(value.z), w(value.w) {}
    explicit                quat(const mat33& matrix);
    explicit                quat(const mat44& matrix);

    constexpr quat          operator*(const quat& rhs) const;
    constexpr quat&         operator*=(const quat& value);
    constexpr quat          operator-() const { return { -x, -y, -z, -w }; }

    friend constexpr vec3   operator*(const vec3& lhs, const quat& rhs);
    friend constexpr vec4   operator*(const vec4& lhs, const quat& rhs);

    constexpr bool          operator==(const quat& value) const;
    constexpr bool          operator!=(const quat& value) const;

    constexpr float&        operator[](size_t index) { return data[index]; }
    constexpr const float&  operator[](size_t index) const { return data[index]; }

    constexpr float         length_squared() const { return sqr(x) + sqr(y) + sqr(z) + sqr(w); }
    float                   length() const { return std::sqrt(length_squared()); }
    quat                    normalized() const;

    constexpr quat          transposed() const { return { -x, -y, -z, w }; }
    constexpr quat          inverted() const;

    bool                    is_finite() const;
    constexpr bool          equal(const quat& rhs, float epsilon = EQUALITY_EPSILON) const;

    union {
        struct {
            float x, y, z, w;
        };

        float data[4]{};
    };
};

quat slerp(const quat& from, quat to, float factor);

constexpr quat quat::operator*(const quat& rhs) const
{
    float a = (w + x) * (rhs.w + rhs.x);
    float b = (z - y) * (rhs.y - rhs.z);
    float c = (x - w) * (rhs.y + rhs.z);
    float d = (y + z) * (rhs.x - rhs.w);
    float e = (x + z) * (rhs.x + rhs.y);
    float f = (x - z) * (rhs.x - rhs.y);
    float g = (w + y) * (rhs.w - rhs.z);
    float h = (w - y) * (rhs.w + rhs.z);

    return {
        a - ( e + f + g + h) * 0.5f,
        -c + ( e - f + g - h) * 0.5f,
        -d + ( e - f - g + h) * 0.5f,
        b + (-e - f + g + h) * 0.5f };
}

constexpr quat& quat::operator*=(const quat& value)
{
    *this = *this * value;
    return *this;
}

constexpr bool quat::operator==(const quat& value) const
{
    return x == value.x && y == value.y && z == value.z && w == value.w;
}

constexpr bool quat::operator!=(const quat& value) const
{
    return x != value.x || y != value.y || z != value.z || w != value.w;
}

constexpr vec3 operator*(const vec3& lhs, const quat& rhs)
{
    vec3 a(rhs.x, rhs.y, rhs.z);
    vec3 b(a.cross(lhs));
    vec3 c(a.cross(b));
    return lhs + ((b * rhs.w) + c) * 2.f;
}

constexpr vec4 operator*(const vec4& lhs, const quat& rhs)
{
    return {vec3(lhs.x, lhs.y, lhs.z) * rhs, 0.f};
}

inline quat quat::normalized() const
{
    float multiplier = 1.f / length();
    return {x * multiplier, y * multiplier, z * multiplier, w * multiplier};
}

constexpr quat quat::inverted() const
{
    float multiplier = 1.f / length_squared();
    quat tr = transposed();
    return {tr.x * multiplier, tr.y * multiplier, tr.z * multiplier, tr.w * multiplier};
}

constexpr bool quat::equal(const quat& rhs, float epsilon) const
{
    return slope::equal(x, rhs.x, epsilon) &&
           slope::equal(y, rhs.y, epsilon) &&
           slope::equal(z, rhs.z, epsilon) &&
           slope::equal(w, rhs.w, epsilon);
}

inline bool quat::is_finite() const
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
}

} // slope
