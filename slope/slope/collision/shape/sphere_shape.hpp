#pragma once
#include "slope/collision/shape/collision_shape.hpp"

namespace slope {

class SphereShape : public TypedCollisionShape<ShapeKind::Sphere> {
public:
    explicit SphereShape(float radius);

    void    set_transform(const Mat44& matrix) final;
    Vec3    support(const Vec3& axis, float bloat, bool normalized) const final;

    float   radius() const { return m_radius; }

private:
    float m_radius = 1.f;
};

inline SphereShape::SphereShape(float radius) : m_radius(radius) {}

inline Vec3 SphereShape::support(const Vec3& axis, float bloat, bool normalized) const
{
    Vec3 norm_axis = normalized ? axis : axis.normalized();
    return m_transform.translation() + norm_axis * (m_radius + bloat);
}

inline void SphereShape::set_transform(const Mat44& matrix)
{
    m_transform = matrix;
    Vec3 delta = {m_radius, m_radius, m_radius};
    m_aabb.reset(matrix.translation() - delta, matrix.translation() + delta);
}

} // slope
