#pragma once
#include "slope/math/vector4.hpp"

namespace slope {

class Mat33;
class Mat44;

class Quat {
public:
    static Quat rotation(const Vec3& axis, float angle);

    static Quat shortest_arc(const Vec3& from, const Vec3& to, const Vec3& spin_axis);

    constexpr Quat() : x(0.f), y(0.f), z(0.f), w(1.f) {}

    constexpr Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    constexpr Quat(const Vec3& value, float w) : x(value.x), y(value.y), z(value.z), w(w) {}

    explicit constexpr Quat(const Vec4& value) : x(value.x), y(value.y), z(value.z), w(value.w) {}

    explicit Quat(const Mat33& matrix);

    explicit Quat(const Mat44& matrix);

    constexpr Quat operator*(const Quat& rhs) const {
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

    constexpr Quat& operator*=(const Quat& value) {
        *this = *this * value;
        return *this;
    }

    constexpr Quat operator-() const {
        return { -x, -y, -z, -w };
    }

    constexpr float& operator[](size_t index) {
        return data[index];
    }

    constexpr const float& operator[](size_t index) const {
        return data[index];
    }

    constexpr bool operator==(const Quat& value) const {
        return x == value.x && y == value.y && z == value.z && w == value.w;
    }

    constexpr bool operator!=(const Quat& value) const {
        return x != value.x || y != value.y || z != value.z || w != value.w;
    }

    friend constexpr Vec3 operator*(const Vec3& lhs, const Quat& rhs) {
        Vec3 a(rhs.x, rhs.y, rhs.z);
        Vec3 b(a.cross(lhs));
        Vec3 c(a.cross(b));
        return lhs + ((b * rhs.w) + c) * 2.f;
    }

    friend constexpr Vec4 operator*(const Vec4& lhs, const Quat& rhs) {
        return { Vec3(lhs.x, lhs.y, lhs.z) * rhs, 0.f };
    }

    constexpr float* begin() {
        return data;
    }

    [[nodiscard]] constexpr const float* begin() const {
        return data;
    }

    constexpr float* end() {
        return data + 4;
    }

    [[nodiscard]] constexpr const float* end() const {
        return data + 4;
    }

    [[nodiscard]] constexpr float length_squared() const {
        return sqr(x) + sqr(y) + sqr(z) + sqr(w);
    }

    [[nodiscard]] float length() const {
        return std::sqrt(length_squared());
    }

    [[nodiscard]] Quat normalized() const {
        float multiplier = 1.f / length();
        return { x * multiplier, y * multiplier, z * multiplier, w * multiplier };
    }

    [[nodiscard]] constexpr Quat transposed() const {
        return { -x, -y, -z, w };
    }

    [[nodiscard]] constexpr Quat inverted() const {
        float multiplier = 1.f / length_squared();
        Quat tr = transposed();
        return { tr.x * multiplier, tr.y * multiplier, tr.z * multiplier, tr.w * multiplier };
    }

    [[nodiscard]] constexpr bool equal(const Quat& rhs, float epsilon = EPSILON) const {
        return  slope::equal(x, rhs.x, epsilon) &&
                slope::equal(y, rhs.y, epsilon) &&
                slope::equal(z, rhs.z, epsilon) &&
                slope::equal(w, rhs.w, epsilon);
    }

    [[nodiscard]] bool isfinite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
    }

    void slerp(const Quat& from, Quat to, float factor);

    union {
        struct {
            float x, y, z, w;
        };

        float data[4]{};
    };
};

inline Quat slerp(const Quat& from, Quat to, float factor) {
    float cos_a = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;

    if (cos_a < 0.f) {
        cos_a = -cos_a;
        to = -to;
    }

    if (cos_a > 0.995f) {
        return Quat(lerp(
                Vec4(from.x, from.y, from.z, from.w),
                Vec4(to.x, to.y, to.z, to.w),
                factor)).normalized();
    }

    factor = factor * 0.5f;

    float a = std::acos(cos_a);
    float b = 1.f / std::sin(a);
    float c = std::sin((1 - factor) * a) * b;
    float d = std::sin(factor * a) * b;

    return Quat(
            c * from.x + d * to.x,
            c * from.y + d * to.y,
            c * from.z + d * to.z,
            c * from.w + d * to.w).normalized();
}

inline void Quat::slerp(const Quat& from, Quat to, float factor)
{
    *this = slope::slerp(from, to, factor);
}

} // slope