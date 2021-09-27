#pragma once
#include "slope/math/math_utils.hpp"
#include "slope/math/vector3.hpp"

namespace slope {

class Quat;

class Vec4 {
public:
    constexpr Vec4() : x(0.f), y(0.f), z(0.f), w(0.f) {}

    explicit constexpr Vec4(float all) : x(all), y(all), z(all), w(all) {}

    constexpr Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    explicit constexpr Vec4(const Vec3& v) : x(v.x), y(v.y), z(v.z), w(1.f) {}

    constexpr Vec4(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}

    constexpr Vec4 operator+(const Vec4& rhs) const {
        return { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
    }

    constexpr Vec4 operator-(const Vec4& rhs) const {
        return { x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w };
    }

    constexpr Vec4 operator*(const Vec4& rhs) const {
        return { x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w };
    }

    constexpr Vec4 operator/(const Vec4& rhs) const {
        return { x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w };
    }

    constexpr Vec4& operator+=(const Vec4& value) {
        x += value.x;
        y += value.y;
        z += value.z;
        w += value.w;

        return *this;
    }

    constexpr Vec4& operator-=(const Vec4& value) {
        x -= value.x;
        y -= value.y;
        z -= value.z;
        w -= value.w;

        return *this;
    }

    constexpr Vec4& operator*=(const Vec4& value) {
        x *= value.x;
        y *= value.y;
        z *= value.z;
        w *= value.w;

        return *this;
    }

    constexpr Vec4& operator/=(const Vec4& value) {
        x /= value.x;
        y /= value.y;
        z /= value.z;
        w /= value.w;

        return *this;
    }

    constexpr Vec4& operator*=(float value) {
        x *= value;
        y *= value;
        z *= value;
        w *= value;

        return *this;
    }

    constexpr Vec4& operator/=(float value) {
        x /= value;
        y /= value;
        z /= value;
        w /= value;

        return *this;
    }

    constexpr float& operator[](size_t index) {
        return data[index];
    }

    constexpr const float& operator[](size_t index) const {
        return data[index];
    }

    constexpr Vec4 operator-() const {
        return { -x, -y, -z, -w };
    }

    constexpr Vec4 operator*(float rhs) const {
        return { x * rhs, y * rhs, z * rhs, w * rhs };
    }

    constexpr Vec4 operator/(float rhs) const {
        return { x / rhs, y / rhs, z / rhs, w / rhs };
    }

    constexpr bool operator==(const Vec4& value) const {
        return x == value.x && y == value.y && z == value.z && w == value.w;
    }

    constexpr bool operator!=(const Vec4& value) const {
        return x != value.x || y != value.y || z != value.z || w != value.w;
    }

    friend constexpr Vec4 operator*(float lhs, const Vec4& rhs) {
        return rhs * lhs;
    }

    float* begin() {
        return data;
    }

    [[nodiscard]] const float* begin() const {
        return data;
    }

    float* end() {
        return data + 4;
    }

    [[nodiscard]] const float *end() const {
        return data + 4;
    }

    [[nodiscard]] constexpr float dot(const Vec4& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
    }

    [[nodiscard]] constexpr Vec4 cross(const Vec4& rhs) const {
        return { y * rhs.z - z * rhs.y,
                 z * rhs.x - x * rhs.z,
                 x * rhs.y - y * rhs.x,
                 1.f };
    }

    [[nodiscard]] constexpr float length_squared() const {
        return sqr(x) + sqr(y) + sqr(z) + sqr(w);
    }

    [[nodiscard]] float length() const {
        return std::sqrt(length_squared());
    }

    [[nodiscard]] constexpr float square_distance(const Vec4& rhs) const {
        return sqr(x - rhs.x) + sqr(y - rhs.y) + sqr(z - rhs.z) + sqr(w - rhs.w);
    }

    [[nodiscard]] float distance(const Vec4& rhs) const {
        return std::sqrt(square_distance(rhs));
    }

    [[nodiscard]] bool isfinite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
    }

    [[nodiscard]] constexpr bool equal(const Vec4& rhs, float epsilon = EPSILON) const {
        return  slope::equal(x, rhs.x, epsilon) &&
                slope::equal(y, rhs.y, epsilon) &&
                slope::equal(z, rhs.z, epsilon) &&
                slope::equal(w, rhs.w, epsilon);
    }

    [[nodiscard]] constexpr bool equal(float rhs, float epsilon = EPSILON) const {
        return  slope::equal(x, rhs, epsilon) &&
                slope::equal(y, rhs, epsilon) &&
                slope::equal(z, rhs, epsilon) &&
                slope::equal(w, rhs, epsilon);
    }

    [[nodiscard]] Vec4 normalized() const;

    [[nodiscard]] constexpr Vec4 reflected(const Vec4& normal) const;

    [[nodiscard]] Vec4 abs() const;

    constexpr void lerp(const Vec4& from, const Vec4& to, float t);

    constexpr void clamp(const Vec4& value, float min, float max);

    constexpr void clamp(const Vec4& value, const Vec4& min, const Vec4& max);

    void min(const Vec4& lhs, const Vec4& rhs);

    void max(const Vec4& lhs, const Vec4& rhs);

    union {
        struct {
            float x, y, z, w;
        };

        float data[3]{};
    };
};

inline Vec4 normalize(const Vec4& value) {
    float multiplier = 1.f / value.length();
    return { value.x * multiplier, value.y * multiplier, value.z * multiplier, value.w * multiplier };
}

inline Vec4 Vec4::normalized() const {
    return normalize(*this);
}

constexpr Vec4 reflect(const Vec4& value, const Vec4& normal) {
    return value - (2.f * value.dot(normal)) * normal;
}

constexpr Vec4 Vec4::reflected(const Vec4& normal) const {
    return reflect(*this, normal);
}

inline Vec4 abs(const Vec4& value) {
    return { std::abs(value.x), std::abs(value.y), std::abs(value.z), std::abs(value.w) };
}

inline Vec4 Vec4::abs() const {
    return slope::abs(*this);
}

constexpr Vec4 lerp(const Vec4& from, const Vec4& to, float factor) {
    return from + (to - from) * factor;
}

constexpr void Vec4::lerp(const Vec4& from, const Vec4& to, float t) {
    *this = slope::lerp(from, to, t);
}

constexpr Vec4 clamp(const Vec4& value, float min, float max) {
    return { clamp(value.x, min, max), clamp(value.y, min, max), clamp(value.z, min, max), clamp(value.w, min, max) };
}

constexpr void Vec4::clamp(const Vec4& value, float min, float max) {
    *this = slope::clamp(value, min, max);
}

constexpr Vec4 clamp(const Vec4& value, const Vec4& min, const Vec4& max) {
    return { clamp(value.x, min.x, max.x), clamp(value.y, min.y, max.y),
             clamp(value.z, min.z, max.z), clamp(value.w, min.w, max.w) };
}

constexpr void Vec4::clamp(const Vec4& value, const Vec4& min, const Vec4& max) {
    *this = slope::clamp(value, min, max);
}

inline Vec4 min(const Vec4& lhs, const Vec4& rhs) {
    return { std::fmin(lhs.x, rhs.x), std::fmin(lhs.y, rhs.y), std::fmin(lhs.z, rhs.z), std::fmin(lhs.w, rhs.w) };
}

inline void Vec4::min(const Vec4& lhs, const Vec4& rhs) {
    *this = slope::min(lhs, rhs);
}

inline Vec4 max(const Vec4& lhs, const Vec4& rhs) {
    return { std::fmax(lhs.x, rhs.x), std::fmax(lhs.y, rhs.y), std::fmax(lhs.z, rhs.z), std::fmax(lhs.w, rhs.w) };
}

inline void Vec4::max(const Vec4& lhs, const Vec4& rhs) {
    *this = slope::max(lhs, rhs);
}

} // slope