#include "slope/collision/shape/capsule_shape.hpp"

namespace slope {

CapsuleShape::CapsuleShape(float radius, float height) : m_radius(radius), m_height(height) {}

void CapsuleShape::set_transform(const Mat44& matrix)
{
    m_transform = matrix;

    Vec3 offset = {0.f, m_height * 0.5f, 0.f};
    m_segment.begin = matrix.apply_point(-offset);
    m_segment.end = matrix.apply_point(offset);

    Vec3 delta = {m_radius, m_radius, m_radius};
    m_aabb.reset(m_segment.begin - delta, m_segment.begin + delta);
    m_aabb.extend(m_segment.end - delta);
    m_aabb.extend(m_segment.end + delta);
}

} // slope
