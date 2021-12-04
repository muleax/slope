#pragma once
#include "slope/collision/shape/collision_shape.hpp"

namespace slope {

class SphereShape : public CollisionShapeImpl<ShapeType::Sphere> {
public:
    explicit SphereShape(float radius);

    Vec3    support(const Vec3& axis, float bloat) const final { return support_impl(axis.normalized(), bloat); }
    Vec3    support_normalized(const Vec3& axis, float bloat) const final { return support_impl(axis, bloat); }
    void    set_transform(const Mat44& matrix) final;

    float   radius() const { return m_radius; }

private:
    Vec3 support_impl(const Vec3& axis, float bloat) const;

    float m_radius = 1.f;
};

inline SphereShape::SphereShape(float radius) : m_radius(radius) {}

inline Vec3 SphereShape::support_impl(const Vec3& axis, float bloat) const
{
    return m_transform.translation() + axis * (m_radius + bloat);
}

inline void SphereShape::set_transform(const Mat44& matrix)
{
    m_transform = matrix;
    Vec3 delta = {m_radius, m_radius, m_radius};
    m_aabb.reset(matrix.translation() - delta, matrix.translation() + delta);
}

} // slope
