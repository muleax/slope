#include "slope/math/quat.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

Quat Quat::rotation(const Vec3& axis, float angle) {
    SL_ASSERT(axis.is_finite());
    SL_ASSERT(std::isfinite(angle));
    SL_ASSERT(slope::equal(axis.length(), 1.f));

    return { std::sin(0.5f * angle) * axis, std::cos(0.5f * angle) };
}

Quat Quat::shortest_arc(const Vec3& from, const Vec3& to, const Vec3& spin_axis) {
    SL_ASSERT(from.isfinite());
    SL_ASSERT(to.isfinite());
    SL_ASSERT(spin_axis.isfinite());
    SL_ASSERT(slope::equal(from.length(), 1.f));
    SL_ASSERT(slope::equal(to.length(), 1.f));
    SL_ASSERT(slope::equal(spin_axis.length(), 1.f));

    float dot_product = from.dot(to);
    if (slope::equal(dot_product, 1.f)) {
        return {};
    } else if (slope::equal(dot_product, -1.f)) {
        return { spin_axis, 0.f };
    } else {
        return Quat(from.cross(to), dot_product + 1.f).normalized();
    }
}

Quat::Quat(const Mat33& matrix) {
    SL_ASSERT(isfinite(matrix));

    float a = matrix._11 - matrix._22 - matrix._33;
    float b = matrix._22 - matrix._11 - matrix._33;
    float c = matrix._33 - matrix._11 - matrix._22;
    float d = matrix._11 + matrix._22 + matrix._33;

    int greatest_index = 3;
    float greatest_index_value = d;

    if (a > greatest_index_value) {
        greatest_index_value = a;
        greatest_index = 0;
    }

    if (b > greatest_index_value) {
        greatest_index_value = b;
        greatest_index = 1;
    }

    if (c > greatest_index_value) {
        greatest_index_value = c;
        greatest_index = 2;
    }

    float greatest_value = std::sqrt(greatest_index_value + 1.f) * 0.5f;
    float multiplier = 0.25f / greatest_value;

    switch (greatest_index) {
        case 0:
            x = greatest_value;
            y = (matrix._12 + matrix._21) * multiplier;
            z = (matrix._31 + matrix._13) * multiplier;
            w = (matrix._23 - matrix._32) * multiplier;
            break;
        case 1:
            x = (matrix._12 + matrix._21) * multiplier;
            y = greatest_value;
            z = (matrix._23 + matrix._32) * multiplier;
            w = (matrix._31 - matrix._13) * multiplier;
            break;
        case 2:
            x = (matrix._31 + matrix._13) * multiplier;
            y = (matrix._23 + matrix._32) * multiplier;
            z = greatest_value;
            w = (matrix._12 - matrix._21) * multiplier;
            break;
        case 3:
            x = (matrix._23 - matrix._32) * multiplier;
            y = (matrix._31 - matrix._13) * multiplier;
            z = (matrix._12 - matrix._21) * multiplier;
            w = greatest_value;
            break;
        default:
            x = 0.f;
            y = 0.f;
            z = 0.f;
            w = 1.f;
    }
}

Quat::Quat(const Mat44& matrix)
        : Quat(Mat33(
                matrix._11, matrix._12, matrix._13,
                matrix._21, matrix._22, matrix._23,
                matrix._31, matrix._32, matrix._33)) {}

} // slope