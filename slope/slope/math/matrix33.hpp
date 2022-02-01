#pragma once
#include "slope/math/matrix22.hpp"
#include "slope/math/vector3.hpp"

namespace slope {

class Quat;

class Mat33 {
public:
    static Mat33 cross(const Vec3& v) {
        return { 0.f, v.z, -v.y,
                 -v.z, 0.f, v.x,
                 v.y, -v.x, 0.f };
    }

    constexpr Mat33()
            : _11(1.f), _12(0.f), _13(0.f)
            , _21(0.f), _22(1.f), _23(0.f)
            , _31(0.f), _32(0.f), _33(1.f) {}

    constexpr Mat33(    float _11, float _12, float _13,
                        float _21, float _22, float _23,
                        float _31, float _32, float _33)
            : _11(_11), _12(_12), _13(_13)
            , _21(_21), _22(_22), _23(_23)
            , _31(_31), _32(_32), _33(_33) {}

    explicit constexpr Mat33(const Mat22& value)
            : _11(value._11), _12(value._12), _13(0.f)
            , _21(value._21), _22(value._22), _23(0.f)
            , _31(0.f),       _32(0.f),       _33(1.f) {}

    constexpr Mat33(const Vec3& r0, const Vec3& r1, const Vec3& r2) : rows{ r0, r1, r2 } {}

    explicit Mat33(const Quat& q);

    constexpr Mat33 operator-() const {
        return {
            -_11, -_12, -_13,
            -_21, -_22, -_23,
            -_31, -_32, -_33 };
    }

    constexpr Mat33 operator+(const Mat33& rhs) const {
        return {
            _11 + rhs._11, _12 + rhs._12, _13 + rhs._13,
            _21 + rhs._21, _22 + rhs._22, _23 + rhs._23,
            _31 + rhs._31, _32 + rhs._32, _33 + rhs._33 };
    }

    constexpr Mat33 operator-(const Mat33& rhs) const {
        return {
            _11 - rhs._11, _12 - rhs._12, _13 - rhs._13,
            _21 - rhs._21, _22 - rhs._22, _23 - rhs._23,
            _31 - rhs._31, _32 - rhs._32, _33 - rhs._33 };
    }

    constexpr Mat33 operator*(const Mat33& rhs) const {
        return {
            _11 * rhs._11 + _12 * rhs._21 + _13 * rhs._31,
            _11 * rhs._12 + _12 * rhs._22 + _13 * rhs._32,
            _11 * rhs._13 + _12 * rhs._23 + _13 * rhs._33,
            _21 * rhs._11 + _22 * rhs._21 + _23 * rhs._31,
            _21 * rhs._12 + _22 * rhs._22 + _23 * rhs._32,
            _21 * rhs._13 + _22 * rhs._23 + _23 * rhs._33,
            _31 * rhs._11 + _32 * rhs._21 + _33 * rhs._31,
            _31 * rhs._12 + _32 * rhs._22 + _33 * rhs._32,
            _31 * rhs._13 + _32 * rhs._23 + _33 * rhs._33 };
    }

    constexpr Mat33 operator*(float rhs) const {
        return {
            _11 * rhs, _12 * rhs, _13 * rhs,
            _21 * rhs, _22 * rhs, _23 * rhs,
            _31 * rhs, _32 * rhs, _33 * rhs };
    }

    constexpr Vec3 operator*(const Vec3& rhs) const {
        return {
            _11 * rhs.x + _12 * rhs.y + _13 * rhs.z,
            _21 * rhs.x + _22 * rhs.y + _23 * rhs.z,
            _31 * rhs.x + _32 * rhs.y + _33 * rhs.z };
    }

    friend constexpr Mat33 operator*(float lhs, const Mat33& rhs) {
        return rhs * lhs;
    }

    friend constexpr Vec3 operator*(const Vec3& lhs, const Mat33& rhs) {
        return rhs * lhs;
    }

    constexpr Mat33 operator/(float rhs) const {
        return *this * (1.f / rhs);
    }

    constexpr Mat33& operator+=(const Mat33& value) {
        return *this = *this + value;
    }

    constexpr Mat33& operator-=(const Mat33& value) {
        return *this = *this - value;
    }

    constexpr Mat33& operator*=(const Mat33& value) {
        return *this = *this * value;
    }

    constexpr Mat33& operator*=(float value) {
        return *this = *this * value;
    }

    constexpr Mat33& operator/=(float value) {
        return *this = *this / value;
    }

    constexpr Vec3& operator[](size_t index) {
        return rows[index];
    }

    constexpr const Vec3& operator[](size_t index) const {
        return rows[index];
    }

    constexpr bool operator==(const Mat33& value) const {
        return _11 == value._11 && _12 == value._12 && _13 == value._13 &&
               _21 == value._21 && _22 == value._22 && _23 == value._23 &&
               _31 == value._31 && _32 == value._32 && _33 == value._33;
    }

    constexpr bool operator!=(const Mat33& value) const {
        return !(*this == value);
    }

    constexpr float* begin() {
        return data;
    }

    constexpr const float* begin() const {
        return data;
    }

    constexpr float* end() {
        return data + 9;
    }

    constexpr const float* end() const {
        return data + 9;
    }

    constexpr float determinant() const {
        return _11 * _11 + _21 * _12 + _31 * _13;
    }

    constexpr Mat33 transposed(const Mat33& value) const {
        return {
            value._11, value._21, value._31,
            value._12, value._22, value._32,
            value._13, value._23, value._33 };
    }

    constexpr Mat33 inverted() const {
        Mat33 result(
            _33 * _22 - _23 * _32,
            _13 * _32 - _33 * _12,
            _23 * _12 - _13 * _22,
            _23 * _31 - _33 * _21,
            _33 * _11 - _13 * _31,
            _13 * _21 - _23 * _11,
            _21 * _32 - _31 * _22,
            _31 * _12 - _11 * _32,
            _11 * _22 - _21 * _12);

        float det = determinant();
        if (slope::equal(det, 0.f)) {
            return {};
        }

        float factor = 1.f / det;
        result._11 *= factor;
        result._12 *= factor;
        result._13 *= factor;
        result._21 *= factor;
        result._22 *= factor;
        result._23 *= factor;
        result._31 *= factor;
        result._32 *= factor;
        result._33 *= factor;

        return result;
    }

    constexpr bool equal(const Mat33& rhs, float epsilon = EQUALITY_EPSILON) const {
        return  slope::equal(_11, rhs._11, epsilon) &&
                slope::equal(_12, rhs._12, epsilon) &&
                slope::equal(_13, rhs._13, epsilon) &&
                slope::equal(_21, rhs._21, epsilon) &&
                slope::equal(_22, rhs._22, epsilon) &&
                slope::equal(_23, rhs._23, epsilon) &&
                slope::equal(_31, rhs._31, epsilon) &&
                slope::equal(_32, rhs._32, epsilon) &&
                slope::equal(_33, rhs._33, epsilon);
    }

    inline bool isfinite() const {
        return std::isfinite(_11) && std::isfinite(_12) && std::isfinite(_13) &&
               std::isfinite(_21) && std::isfinite(_22) && std::isfinite(_23) &&
               std::isfinite(_31) && std::isfinite(_32) && std::isfinite(_33);
    }

    union {
        struct {
            float _11, _12, _13;
            float _21, _22, _23;
            float _31, _32, _33;
        };

        Vec3 rows[3];
        float data[9];
    };
};

} // namespace kw