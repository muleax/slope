#include "slope/collision/shape/box_shape.hpp"

namespace slope {

BoxShape::BoxShape(const Vec3& dimensions) : m_half_dimensions(dimensions * 0.5f) {}

void BoxShape::set_transform(const Mat44& matrix)
{
    m_transform = matrix;

    Vec3 dv;
    for (int i = 0; i < 3; i++) {
        m_principal_axes[i] = matrix.apply_to_unit_axis(i);
        m_extents[i] = m_principal_axes[i] * m_half_dimensions[i];

        for (int j = 0; j < 3; j++)
            dv[j] += fabs(m_extents[i][j]);
    }

    m_aabb.reset(matrix.translation() - dv, matrix.translation() + dv);
}

Vec3 BoxShape::support_point(const Vec3& axis, float bloat) const
{
// TODO: consider getting rid of normalization
    auto norm_axis = axis.normalized();

    Vec3 offs;
    for (int i = 0; i < 3; i++) {
        float dot = norm_axis.dot(m_extents[i]);
        offs += m_extents[i] * signf(dot);
    }

    return m_transform.translation() + offs;
}

float BoxShape::get_support_face(const Vec3& axis, Vector<Vec3>& out_support, Vec3& out_face_normal) const {
    int best_axis = -1;
    float best_proximity = -FLOAT_MAX;
    float best_sgn = 0.f;

    for (int i = 0; i < 3; i++) {
        float proximity = axis.dot(m_principal_axes[i]);
        float s = signf(proximity);
        proximity *= s;
        if (proximity > best_proximity) {
            best_axis = i;
            best_proximity = proximity;
            best_sgn = s;
        }
    }

    Vec3 offset = m_transform.translation() + m_extents[best_axis] * best_sgn;
    auto u = best_sgn * m_extents[(best_axis + 1) % 3];
    auto v = m_extents[(best_axis + 2) % 3];

    out_support.push_back(offset + u + v);
    out_support.push_back(offset + u - v);
    out_support.push_back(offset - u - v);
    out_support.push_back(offset - u + v);

    out_face_normal = best_sgn * m_principal_axes[best_axis];

    return best_proximity;
}

} // slope
