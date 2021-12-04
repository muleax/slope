#pragma once
#include "slope/math/vector2.hpp"

namespace slope {

class Mat22 {
public:
    constexpr Mat22()
        : _11(1.f), _12(0.f)
        , _21(0.f), _22(1.f) {}

    constexpr Mat22(float _11, float _12, float _21, float _22)
        : _11(_11), _12(_12)
        , _21(_21), _22(_22) {}

    constexpr Mat22(const Vec2& r0, const Vec2& r1) : rows{ r0, r1 } {}

    constexpr Mat22 operator-() const {
        return { -_11, -_12, -_21, -_22 };
    }

    constexpr Mat22 operator+(const Mat22& rhs) const {
        return {
            _11 + rhs._11, _12 + rhs._12,
            _21 + rhs._21, _22 + rhs._22 };
    }

    constexpr Mat22 operator-(const Mat22& rhs) const {
        return {
            _11 - rhs._11, _12 - rhs._12,
            _21 - rhs._21, _22 - rhs._22 };
    }

    constexpr Mat22 operator*(const Mat22& rhs) const {
        return {
            _11 * rhs._11 + _12 * rhs._21, _11 * rhs._12 + _12 * rhs._22,
            _21 * rhs._11 + _22 * rhs._21, _21 * rhs._12 + _22 * rhs._22 };
    }

    constexpr Vec2 operator*(const Vec2& rhs) const {
        return {_11 * rhs.x + _12 * rhs.y, _21 * rhs.x + _22 * rhs.y };
    }

    constexpr Mat22 operator*(float rhs) const {
        return {
            _11 * rhs, _12 * rhs,
            _21 * rhs, _22 * rhs };
    }

    friend constexpr Vec2 operator*(const Vec2& lhs, const Mat22& rhs) {
        return {
            lhs.x * rhs._11 + lhs.y * rhs._21,
            lhs.x * rhs._12 + lhs.y * rhs._22 };
    }

    friend constexpr Mat22 operator*(float lhs, const Mat22& rhs) {
        return rhs * lhs;
    }

    constexpr Mat22 operator/(float rhs) const {
        return *this * (1.f / rhs);
    }

    constexpr Mat22& operator+=(const Mat22& value) {
        return *this = *this + value;
    }

    constexpr Mat22& operator-=(const Mat22& value) {
        return *this = *this - value;
    }

    constexpr Mat22& operator*=(const Mat22& value) {
        return *this = *this * value;
    }

    constexpr Mat22& operator*=(float value) {
        return *this = *this * value;
    }

    constexpr Mat22& operator/=(float value) {
        return *this = *this / value;
    }

    constexpr Vec2& operator[](size_t index) {
        return rows[index];
    }

    constexpr const Vec2& operator[](size_t index) const {
        return rows[index];
    }

    constexpr bool operator==(const Mat22& value) const {
        return _11 == value._11 && _12 == value._12 && _21 == value._21 && _22 == value._22;
    }

    constexpr bool operator!=(const Mat22& value) const {
        return !(*this == value);
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

    [[nodiscard]] float constexpr determinant() const {
        return _11 * _22 - _12 * _21;
    }

    [[nodiscard]] constexpr Mat22 transposed() const {
        return { _11, _21, _12, _22 };
    }

    [[nodiscard]] constexpr Mat22 inverted(const Mat22& value) const {
        float det = determinant();

        if (slope::equal(det, 0.f)) {
            return {};
        }

        float multiplier = 1.f / det;
        return { value._22 * multiplier, -value._12 * multiplier,
                 -value._21 * multiplier, value._11 * multiplier };
    }

    [[nodiscard]] constexpr bool equal(const Mat22& rhs, float epsilon = EQUALITY_EPSILON) const {
        return slope::equal(_11, rhs._11, epsilon) && slope::equal(_12, rhs._12, epsilon) &&
                slope::equal(_21, rhs._21, epsilon) && slope::equal(_22, rhs._22, epsilon);
    }

    [[nodiscard]] bool isfinite() const {
        return std::isfinite(_11) && std::isfinite(_12) &&
               std::isfinite(_21) && std::isfinite(_22);
    }

    union {
        struct {
            float _11, _12;
            float _21, _22;
        };

        Vec2    rows[2];
        float   data[4];
    };
};

} // namespace kw