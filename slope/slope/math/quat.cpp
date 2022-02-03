#include "slope/math/quat.hpp"
#include "slope/math/matrix44.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

quat quat::rotation(const vec3& axis, float angle)
{
    SL_ASSERT(axis.is_finite());
    SL_ASSERT(std::isfinite(angle));
    SL_ASSERT(slope::equal(axis.length(), 1.f));

    return {std::sin(0.5f * angle) * axis, std::cos(0.5f * angle)};
}

quat quat::shortest_arc(const vec3& from, const vec3& to, const vec3& spin_axis)
{
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
        return {spin_axis, 0.f};
    } else {
        return quat(from.cross(to), dot_product + 1.f).normalized();
    }
}

quat::quat(const mat33& matrix)
{
    SL_ASSERT(isfinite(matrix));

    float a = matrix._00 - matrix._11 - matrix._22;
    float b = matrix._11 - matrix._00 - matrix._22;
    float c = matrix._22 - matrix._00 - matrix._11;
    float d = matrix._00 + matrix._11 + matrix._22;

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
        y = (matrix._01 + matrix._10) * multiplier;
        z = (matrix._20 + matrix._02) * multiplier;
        w = (matrix._12 - matrix._21) * multiplier;
        break;
    case 1:
        x = (matrix._01 + matrix._10) * multiplier;
        y = greatest_value;
        z = (matrix._12 + matrix._21) * multiplier;
        w = (matrix._20 - matrix._02) * multiplier;
        break;
    case 2:
        x = (matrix._20 + matrix._02) * multiplier;
        y = (matrix._12 + matrix._21) * multiplier;
        z = greatest_value;
        w = (matrix._01 - matrix._10) * multiplier;
        break;
    case 3:
        x = (matrix._12 - matrix._21) * multiplier;
        y = (matrix._20 - matrix._02) * multiplier;
        z = (matrix._01 - matrix._10) * multiplier;
        w = greatest_value;
        break;
    default:
        x = 0.f;
        y = 0.f;
        z = 0.f;
        w = 1.f;
    }
}

quat::quat(const mat44& matrix)
    : quat(mat33(
    matrix._00, matrix._01, matrix._02,
    matrix._10, matrix._11, matrix._12,
    matrix._20, matrix._21, matrix._22)) {}

quat slerp(const quat& from, quat to, float factor)
{
    float cos_a = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;

    if (cos_a < 0.f) {
        cos_a = -cos_a;
        to = -to;
    }

    if (cos_a > 0.995f) {
        return quat(lerp(
            vec4(from.x, from.y, from.z, from.w),
            vec4(to.x, to.y, to.z, to.w),
            factor)).normalized();
    }

    factor = factor * 0.5f;

    float a = std::acos(cos_a);
    float b = 1.f / std::sin(a);
    float c = std::sin((1 - factor) * a) * b;
    float d = std::sin(factor * a) * b;

    return quat(
        c * from.x + d * to.x,
        c * from.y + d * to.y,
        c * from.z + d * to.z,
        c * from.w + d * to.w).normalized();
}

} // slope
