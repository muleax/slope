#pragma once
#include "slope/math/matrix33.hpp"
#include "slope/math/vector4.hpp"

namespace slope
{

class Mat44 {
public:
    static Mat44 identity() { return {}; }
    static Mat44 rotation(const Vec3& axis, float angle);
    static Mat44 scale(const Vec3& scale);
    static Mat44 translate(const Vec3& translation);

    constexpr Mat44()
        : _11(1.f), _12(0.f), _13(0.f), _14(0.f)
        , _21(0.f), _22(1.f), _23(0.f), _24(0.f)
        , _31(0.f), _32(0.f), _33(1.f), _34(0.f)
        , _41(0.f), _42(0.f), _43(0.f), _44(1.f) {}

    constexpr Mat44(
            float _11, float _12, float _13, float _14,
            float _21, float _22, float _23, float _24,
            float _31, float _32, float _33, float _34,
            float _41, float _42, float _43, float _44)
        : _11(_11), _12(_12), _13(_13), _14(_14)
        , _21(_21), _22(_22), _23(_23), _24(_24)
        , _31(_31), _32(_32), _33(_33), _34(_34)
        , _41(_41), _42(_42), _43(_43), _44(_44) {}

    explicit constexpr Mat44(const Mat22& value)
        : _11(value._11), _12(value._12), _13(0.f), _14(0.f)
        , _21(value._21), _22(value._22), _23(0.f), _24(0.f)
        , _31(0.f), _32(0.f), _33(1.f), _34(0.f)
        , _41(0.f), _42(0.f), _43(0.f), _44(1.f)
    {
    }

    explicit constexpr Mat44(const Mat33& value)
        : _11(value._11), _12(value._12), _13(value._13), _14(0.f)
        , _21(value._21), _22(value._22), _23(value._23), _24(0.f)
        , _31(value._31), _32(value._32), _33(value._33), _34(0.f)
        , _41(0.f), _42(0.f), _43(0.f), _44(1.f)
    {
    }

    constexpr Mat44(const Vec4& r0, const Vec4& r1, const Vec4& r2, const Vec4& r3)
        : rows{ r0, r1, r2, r3 } {}

    explicit Mat44(const Quat& q);

    constexpr Mat44 operator-() const {
        return {
            -_11, -_12, -_13, -_14,
            -_21, -_22, -_23, -_24,
            -_31, -_32, -_33, -_34,
            -_41, -_42, -_43, -_44 };
    }

    constexpr Mat44 operator+(const Mat44& rhs) const {
        return {
            _11 + rhs._11, _12 + rhs._12, _13 + rhs._13, _14 + rhs._14,
            _21 + rhs._21, _22 + rhs._22, _23 + rhs._23, _24 + rhs._24,
            _31 + rhs._31, _32 + rhs._32, _33 + rhs._33, _34 + rhs._34,
            _41 + rhs._41, _42 + rhs._42, _43 + rhs._43, _44 + rhs._44 };
    }

    constexpr Mat44 operator-(const Mat44& rhs) const {
        return {
            _11 - rhs._11, _12 - rhs._12, _13 - rhs._13, _14 - rhs._14,
            _21 - rhs._21, _22 - rhs._22, _23 - rhs._23, _24 - rhs._24,
            _31 - rhs._31, _32 - rhs._32, _33 - rhs._33, _34 - rhs._34,
            _41 - rhs._41, _42 - rhs._42, _43 - rhs._43, _44 - rhs._44 };
    }

    constexpr Mat44 operator*(const Mat44& rhs) const {
        return {
            _11 * rhs._11 + _12 * rhs._21 + _13 * rhs._31 + _14 * rhs._41,
            _11 * rhs._12 + _12 * rhs._22 + _13 * rhs._32 + _14 * rhs._42,
            _11 * rhs._13 + _12 * rhs._23 + _13 * rhs._33 + _14 * rhs._43,
            _11 * rhs._14 + _12 * rhs._24 + _13 * rhs._34 + _14 * rhs._44,
            _21 * rhs._11 + _22 * rhs._21 + _23 * rhs._31 + _24 * rhs._41,
            _21 * rhs._12 + _22 * rhs._22 + _23 * rhs._32 + _24 * rhs._42,
            _21 * rhs._13 + _22 * rhs._23 + _23 * rhs._33 + _24 * rhs._43,
            _21 * rhs._14 + _22 * rhs._24 + _23 * rhs._34 + _24 * rhs._44,
            _31 * rhs._11 + _32 * rhs._21 + _33 * rhs._31 + _34 * rhs._41,
            _31 * rhs._12 + _32 * rhs._22 + _33 * rhs._32 + _34 * rhs._42,
            _31 * rhs._13 + _32 * rhs._23 + _33 * rhs._33 + _34 * rhs._43,
            _31 * rhs._14 + _32 * rhs._24 + _33 * rhs._34 + _34 * rhs._44,
            _41 * rhs._11 + _42 * rhs._21 + _43 * rhs._31 + _44 * rhs._41,
            _41 * rhs._12 + _42 * rhs._22 + _43 * rhs._32 + _44 * rhs._42,
            _41 * rhs._13 + _42 * rhs._23 + _43 * rhs._33 + _44 * rhs._43,
            _41 * rhs._14 + _42 * rhs._24 + _43 * rhs._34 + _44 * rhs._44 };
    }

    constexpr Mat44 operator*(float rhs) const {
        return {
            _11 * rhs, _12 * rhs, _13 * rhs, _14 * rhs,
            _21 * rhs, _22 * rhs, _23 * rhs, _24 * rhs,
            _31 * rhs, _32 * rhs, _33 * rhs, _34 * rhs,
            _41 * rhs, _42 * rhs, _43 * rhs, _44 * rhs };
    }

    constexpr Vec3 operator*(const Vec3& rhs)  const {
        return {
            _11 * rhs.x + _12 * rhs.y + _13 * rhs.z,
            _21 * rhs.x + _22 * rhs.y + _23 * rhs.z,
            _31 * rhs.x + _32 * rhs.y + _33 * rhs.z };
    }

    constexpr Vec4 operator*(const Vec4& rhs) const {
        return {
            _11 * rhs.x + _12 * rhs.y + _13 * rhs.z + _14 * rhs.w,
            _21 * rhs.x + _22 * rhs.y + _23 * rhs.z + _24 * rhs.w,
            _31 * rhs.x + _32 * rhs.y + _33 * rhs.z + _34 * rhs.w,
            _41 * rhs.x + _42 * rhs.y + _43 * rhs.z + _44 * rhs.w};
    }

    friend constexpr Mat44 operator*(float lhs, const Mat44& rhs) {
        return rhs * lhs;
    }

    friend constexpr Vec4 operator*(const Vec4& lhs, const Mat44& rhs) {
        return rhs * lhs;
    }

    friend constexpr Vec3 operator*(const Vec3& lhs, const Mat44& rhs) {
        return rhs * lhs;
    }

    constexpr Mat44 operator/(float rhs) const {
        return *this * (1.f / rhs);
    }

    constexpr Mat44& operator+=(const Mat44& value) {
        return *this = *this + value;
    }

    constexpr Mat44& operator-=(const Mat44& value) {
        return *this = *this - value;
    }

    constexpr Mat44& operator*=(const Mat44& value) {
        return *this = *this * value;
    }

    constexpr Mat44& operator*=(float value) {
        return *this = *this * value;
    }

    constexpr Mat44& operator/=(float value) {
        return *this = *this / value;
    }

    constexpr Vec4& operator[](size_t index) {
        return rows[index];
    }

    Vec3& as_vec3(size_t index) {
        return *reinterpret_cast<Vec3*>(rows + index);
    }

    constexpr const Vec4& operator[](size_t index) const {
        return rows[index];
    }

    constexpr bool operator==(const Mat44& value) const {
        return  _11 == value._11 && _12 == value._12 && _13 == value._13 && _14 == value._14 &&
                _21 == value._21 && _22 == value._22 && _23 == value._23 && _24 == value._24 &&
                _31 == value._31 && _32 == value._32 && _33 == value._33 && _34 == value._34 &&
                _41 == value._41 && _42 == value._42 && _43 == value._43 && _44 == value._44;
    }

    constexpr bool operator!=(const Mat44& value) const {
        return !(*this == value);
    }

    constexpr float* begin() {
        return data;
    }

    [[nodiscard]] constexpr const float* begin() const {
        return data;
    }

    constexpr float* end() {
        return data + 16;
    }

    [[nodiscard]] constexpr const float* end() const {
        return data + 16;
    }

    [[nodiscard]] constexpr Mat44 transposed() const {
        return {
            _11, _21, _31, _41,
            _12, _22, _32, _42,
            _13, _23, _33, _43,
            _14, _24, _34, _44 };
    }

    [[nodiscard]] constexpr Vec3 apply_point(const Vec3& point) const {
        return {
            point.x * _11 + point.y * _21 + point.z * _31 + _41,
            point.x * _12 + point.y * _22 + point.z * _32 + _42,
            point.x * _13 + point.y * _23 + point.z * _33 + _43 };
    }

    [[nodiscard]] constexpr Vec3 apply_normal(const Vec3& normal) const {
        return {
                normal.x * _11 + normal.y * _21 + normal.z * _31,
                normal.x * _12 + normal.y * _22 + normal.z * _32,
                normal.x * _13 + normal.y * _23 + normal.z * _33 };
    }

    [[nodiscard]] const Vec3& translation() const {
        return *reinterpret_cast<const Vec3*>(rows + 3);
    }

    [[nodiscard]] const Vec3& apply_to_unit_axis(uint32_t axis) const {
        return *reinterpret_cast<const Vec3*>(rows + axis);
    }

    void set_translation(const Vec3& translation) {
        rows[3].x = translation.x;
        rows[3].y = translation.y;
        rows[3].z = translation.z;
    }

    [[nodiscard]] Mat44 inverted_rot_pos() const;

    [[nodiscard]] Mat44 inverted() const;

    [[nodiscard]] constexpr bool equal(const Mat44& rhs, float epsilon = EPSILON) const {
        return  slope::equal(_11, rhs._11, epsilon) &&
                slope::equal(_12, rhs._12, epsilon) &&
                slope::equal(_13, rhs._13, epsilon) &&
                slope::equal(_14, rhs._14, epsilon) &&
                slope::equal(_21, rhs._21, epsilon) &&
                slope::equal(_22, rhs._22, epsilon) &&
                slope::equal(_23, rhs._23, epsilon) &&
                slope::equal(_24, rhs._24, epsilon) &&
                slope::equal(_31, rhs._31, epsilon) &&
                slope::equal(_32, rhs._32, epsilon) &&
                slope::equal(_33, rhs._33, epsilon) &&
                slope::equal(_34, rhs._34, epsilon) &&
                slope::equal(_41, rhs._41, epsilon) &&
                slope::equal(_42, rhs._42, epsilon) &&
                slope::equal(_43, rhs._43, epsilon) &&
                slope::equal(_44, rhs._44, epsilon);
    }

    [[nodiscard]] bool isfinite() const {
        return std::isfinite(_11) && std::isfinite(_12) && std::isfinite(_13) && std::isfinite(_14) &&
               std::isfinite(_21) && std::isfinite(_22) && std::isfinite(_23) && std::isfinite(_24) &&
               std::isfinite(_31) && std::isfinite(_32) && std::isfinite(_33) && std::isfinite(_34) &&
               std::isfinite(_41) && std::isfinite(_42) && std::isfinite(_43) && std::isfinite(_44);
    }

    float determinant() const {
        float   det  = _11 * (_22 * _33 - _23 * _32);
                det -= _12 * (_21 * _33 - _23 * _31);
                det += _13 * (_21 * _32 - _22 * _31);
        return det;
    }

    union {
        struct {
            float _11, _12, _13, _14;
            float _21, _22, _23, _24;
            float _31, _32, _33, _34;
            float _41, _42, _43, _44;
        };

        struct {
            float __00, __01, __02, __03;
            float __10, __11, __12, __13;
            float __20, __21, __22, __23;
            float __30, __31, __32, __33;
        };

        Vec4 rows[4];
        float data[16];
    };
};

} // slope
