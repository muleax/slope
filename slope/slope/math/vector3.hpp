#pragma once
#include "slope/math/math_utils.hpp"

namespace slope {

class Vec3 {
public:
    static Vec3 zero() { return {}; }

    constexpr Vec3() : x(0.f), y(0.f), z(0.f) {}

    explicit constexpr Vec3(float all) : x(all), y(all), z(all) {}

    constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    constexpr Vec3 operator+(const Vec3& rhs) const {
        return { x + rhs.x, y + rhs.y, z + rhs.z };
    }

    constexpr Vec3 operator-(const Vec3& rhs) const {
        return { x - rhs.x, y - rhs.y, z - rhs.z };
    }

    constexpr Vec3 operator*(const Vec3& rhs) const {
        return cross(rhs);
    }

    constexpr Vec3 operator/(const Vec3& rhs) const {
        return { x / rhs.x, y / rhs.y, z / rhs.z };
    }

    constexpr Vec3& operator+=(const Vec3& value) {
        x += value.x;
        y += value.y;
        z += value.z;

        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& value) {
        x -= value.x;
        y -= value.y;
        z -= value.z;

        return *this;
    }

    constexpr Vec3& operator/=(const Vec3& value) {
        x /= value.x;
        y /= value.y;
        z /= value.z;

        return *this;
    }

    constexpr Vec3& operator*=(float value) {
        x *= value;
        y *= value;
        z *= value;

        return *this;
    }

    constexpr Vec3& operator/=(float value) {
        x /= value;
        y /= value;
        z /= value;

        return *this;
    }

    constexpr float& operator[](size_t index) {
        return data[index];
    }

    constexpr const float& operator[](size_t index) const {
        return data[index];
    }

    constexpr Vec3 operator-() const {
        return { -x, -y, -z };
    }

    constexpr Vec3 operator*(float rhs) const {
        return { x * rhs, y * rhs, z * rhs };
    }

    constexpr Vec3 operator/(float rhs) const {
        return { x / rhs, y / rhs, z / rhs };
    }

    constexpr bool operator==(const Vec3& value) const {
        return x == value.x && y == value.y && z == value.z;
    }

    constexpr bool operator!=(const Vec3& value) const {
        return x != value.x || y != value.y || z != value.z;
    }

    friend constexpr Vec3 operator*(float lhs, const Vec3& rhs) {
        return rhs * lhs;
    }

    float* begin() {
        return data;
    }

    [[nodiscard]] const float* begin() const {
        return data;
    }

    float* end() {
        return data + 3;
    }

    [[nodiscard]] const float *end() const {
        return data + 3;
    }

    [[nodiscard]] constexpr float dot(const Vec3& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr Vec3 cross(const Vec3& rhs) const {
        return { y * rhs.z - z * rhs.y,
                 z * rhs.x - x * rhs.z,
                 x * rhs.y - y * rhs.x };
    }

    [[nodiscard]] constexpr float length_squared() const {
        return sqr(x) + sqr(y) + sqr(z);
    }

    [[nodiscard]] float length() const {
        return std::sqrt(length_squared());
    }

    [[nodiscard]] constexpr float square_distance(const Vec3& rhs) const {
        return sqr(x - rhs.x) + sqr(y - rhs.y) + sqr(z - rhs.z);
    }

    [[nodiscard]] float distance(const Vec3& rhs) const {
        return std::sqrt(square_distance(rhs));
    }

    [[nodiscard]] bool isfinite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    [[nodiscard]] constexpr bool equal(const Vec3& rhs, float epsilon = EQUALITY_EPSILON) const {
        return slope::equal(x, rhs.x, epsilon) && slope::equal(y, rhs.y, epsilon)  && slope::equal(z, rhs.z, epsilon);
    }

    [[nodiscard]] constexpr bool equal(float rhs, float epsilon = EQUALITY_EPSILON) const {
        return slope::equal(x, rhs, epsilon) && slope::equal(y, rhs, epsilon) && slope::equal(z, rhs, epsilon);
    }

    [[nodiscard]] Vec3 normalized() const;

    [[nodiscard]] constexpr Vec3 reflected(const Vec3& normal) const;

    [[nodiscard]] Vec3 abs() const;

    constexpr void lerp(const Vec3& from, const Vec3& to, float t);

    constexpr void clamp(const Vec3& value, float min, float max);

    constexpr void clamp(const Vec3& value, const Vec3& min, const Vec3& max);

    void min(const Vec3& lhs, const Vec3& rhs);

    void max(const Vec3& lhs, const Vec3& rhs);

    void set_zero() {
        x = 0.f;
        y = 0.f;
        z = 0.f;
    }

    union {
        struct {
            float x, y, z;
        };

        float data[3]{};
    };
};

inline Vec3 normalize(const Vec3& value) {
    float multiplier = 1.f / value.length();
    return { value.x * multiplier, value.y * multiplier, value.z * multiplier };
}

inline Vec3 Vec3::normalized() const {
    return normalize(*this);
}

constexpr Vec3 reflect(const Vec3& value, const Vec3& normal) {
    return value - (2.f * value.dot(normal)) * normal;
}

constexpr Vec3 Vec3::reflected(const Vec3& normal) const {
    return reflect(*this, normal);
}

inline Vec3 abs(const Vec3& value) {
    return { std::abs(value.x), std::abs(value.y), std::abs(value.z) };
}

inline Vec3 Vec3::abs() const {
    return slope::abs(*this);
}

constexpr Vec3 lerp(const Vec3& from, const Vec3& to, float factor) {
    return from + (to - from) * factor;
}

constexpr void Vec3::lerp(const Vec3& from, const Vec3& to, float t) {
    *this = slope::lerp(from, to, t);
}

inline Vec3 min(const Vec3& lhs, const Vec3& rhs) {
    return { std::fmin(lhs.x, rhs.x), std::fmin(lhs.y, rhs.y), std::fmin(lhs.z, rhs.z) };
}

inline void Vec3::min(const Vec3& lhs, const Vec3& rhs) {
    *this = slope::min(lhs, rhs);
}

inline Vec3 max(const Vec3& lhs, const Vec3& rhs) {
    return { std::fmax(lhs.x, rhs.x), std::fmax(lhs.y, rhs.y), std::fmax(lhs.z, rhs.z) };
}

inline void Vec3::max(const Vec3& lhs, const Vec3& rhs) {
    *this = slope::max(lhs, rhs);
}

} // slope