#pragma once
#include "slope/math/math_utils.hpp"

namespace slope {

class Vec2 {
public:
    constexpr Vec2() : x(0.f), y(0.f) {}

    explicit constexpr Vec2(float all) : x(all), y(all) {}

    constexpr Vec2(float x, float y) : x(x), y(y) {}

    constexpr Vec2 operator+(const Vec2& rhs) const {
        return { x + rhs.x, y + rhs.y };
    }

    constexpr Vec2 operator-(const Vec2& rhs) const {
        return { x - rhs.x, y - rhs.y };
    }

    constexpr Vec2 operator*(const Vec2& rhs) const {
        return { x * rhs.x, y * rhs.y };
    }

    constexpr Vec2 operator/(const Vec2& rhs) const {
        return { x / rhs.x, y / rhs.y };
    }

    constexpr Vec2& operator+=(const Vec2& value) {
        x += value.x;
        y += value.y;

        return *this;
    }

    constexpr Vec2& operator-=(const Vec2& value) {
        x -= value.x;
        y -= value.y;

        return *this;
    }

    constexpr Vec2& operator*=(const Vec2& value) {
        x *= value.x;
        y *= value.y;

        return *this;
    }

    constexpr Vec2& operator/=(const Vec2& value) {
        x /= value.x;
        y /= value.y;

        return *this;
    }

    constexpr Vec2& operator*=(float value) {
        x *= value;
        y *= value;

        return *this;
    }

    constexpr Vec2& operator/=(float value) {
        float rcp = 1.f / value;
        x *= rcp;
        y *= rcp;

        return *this;
    }

    constexpr float& operator[](size_t index) {
        return data[index];
    }

    constexpr const float& operator[](size_t index) const {
        return data[index];
    }

    constexpr Vec2 operator-() const {
        return { -x, -y };
    }

    constexpr Vec2 operator*(float rhs) const {
        return { x * rhs, y * rhs };
    }

    constexpr Vec2 operator/(float rhs) const {
        return { x / rhs, y / rhs };
    }

    constexpr bool operator==(const Vec2& value) const {
        return x == value.x && y == value.y;
    }

    constexpr bool operator!=(const Vec2& value) const {
        return x != value.x || y != value.y;
    }

    friend constexpr Vec2 operator*(float lhs, const Vec2& rhs) {
        return rhs * lhs;
    }

    float* begin() {
        return data;
    }

    [[nodiscard]] const float* begin() const {
        return data;
    }

    float* end() {
        return data + 2;
    }

    [[nodiscard]] const float *end() const {
        return data + 2;
    }

    [[nodiscard]] constexpr float dot(const Vec2& rhs) const {
        return x * rhs.x + y * rhs.y;
    }

    [[nodiscard]] constexpr float length_squared() const {
        return sqr(x) + sqr(y);
    }

    [[nodiscard]] float length() const {
        return std::sqrt(length_squared());
    }

    [[nodiscard]] constexpr float square_distance(const Vec2& rhs) const {
        return sqr(x - rhs.x) + sqr(y - rhs.y);
    }

    [[nodiscard]] float distance(const Vec2& rhs) const {
        return std::sqrt(square_distance(rhs));
    }

    [[nodiscard]] bool isfinite() const {
        return std::isfinite(x) && std::isfinite(y);
    }

    [[nodiscard]] constexpr bool equal(const Vec2& rhs, float epsilon = EQUALITY_EPSILON) const {
        return slope::equal(x, rhs.x, epsilon) && slope::equal(y, rhs.y, epsilon);
    }

    [[nodiscard]] constexpr bool equal(float rhs, float epsilon = EQUALITY_EPSILON) const {
        return slope::equal(x, rhs, epsilon) && slope::equal(y, rhs, epsilon);
    }

    [[nodiscard]] Vec2 normalized() const;

    [[nodiscard]] constexpr Vec2 reflected(const Vec2& normal) const;

    [[nodiscard]] Vec2 abs() const;

    constexpr void lerp(const Vec2& from, const Vec2& to, float t);

    constexpr void clamp(const Vec2& value, float min, float max);

    constexpr void clamp(const Vec2& value, const Vec2& min, const Vec2& max);

    void min(const Vec2& lhs, const Vec2& rhs);

    void max(const Vec2& lhs, const Vec2& rhs);

    void set_zero() {
        x = 0.f;
        y = 0.f;
    }

    bool is_zero() const {
        return x == 0.f && y == 0.f;
    }

    union {
        struct {
            float x, y;
        };

        float data[2]{};
    };
};

inline Vec2 normalize(const Vec2& value) {
    float multiplier = 1.f / value.length();
    return { value.x * multiplier, value.y * multiplier };
}

inline Vec2 Vec2::normalized() const {
    return normalize(*this);
}

constexpr Vec2 reflect(const Vec2& value, const Vec2& normal) {
    return value - (2.f * value.dot(normal)) * normal;
}

constexpr Vec2 Vec2::reflected(const Vec2& normal) const {
    return reflect(*this, normal);
}

inline Vec2 abs(const Vec2& value) {
    return { std::abs(value.x), std::abs(value.y) };
}

inline Vec2 Vec2::abs() const {
    return slope::abs(*this);
}

constexpr Vec2 lerp(const Vec2& from, const Vec2& to, float factor) {
    return from + (to - from) * factor;
}

constexpr void Vec2::lerp(const Vec2& from, const Vec2& to, float t) {
    *this = slope::lerp(from, to, t);
}

inline Vec2 min(const Vec2& lhs, const Vec2& rhs) {
    return { std::fmin(lhs.x, rhs.x), std::fmin(lhs.y, rhs.y) };
}

inline void Vec2::min(const Vec2& lhs, const Vec2& rhs) {
    *this = slope::min(lhs, rhs);
}

inline Vec2 max(const Vec2& lhs, const Vec2& rhs) {
    return { std::fmax(lhs.x, rhs.x), std::fmax(lhs.y, rhs.y) };
}

inline void Vec2::max(const Vec2& lhs, const Vec2& rhs) {
    *this = slope::max(lhs, rhs);
}

} // slope