#include "slope/math/matrix22.hpp"

namespace slope {

mat22 mat22::transposed() const
{
    return {
        _00, _10,
        _01, _11 };
}

mat22 mat22::inverted(const mat22& value) const
{
    float det = determinant();

    if (slope::equal(det, 0.f)) {
        return {};
    }

    float multiplier = 1.f / det;
    return {value._11 * multiplier, -value._01 * multiplier,
            -value._10 * multiplier, value._00 * multiplier};
}

bool mat22::equal(const mat22& rhs, float epsilon) const
{
    return slope::equal(_00, rhs._00, epsilon) && slope::equal(_01, rhs._01, epsilon) &&
           slope::equal(_10, rhs._10, epsilon) && slope::equal(_11, rhs._11, epsilon);
}

bool mat22::is_finite() const
{
    return std::isfinite(_00) && std::isfinite(_01) && std::isfinite(_10) && std::isfinite(_11);
}

} // slope
