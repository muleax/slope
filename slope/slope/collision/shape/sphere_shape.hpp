#pragma once
#include "slope/collision/shape/collision_shape.hpp"

namespace slope {

class SphereShape : public CollisionShapeImpl<ShapeType::Sphere> {
public:
    explicit SphereShape(float radius);

    Vec3    support_point(const Vec3& axis, float bloat) const final;
    void    set_transform(const Mat44& matrix) final;

    float   radius() const { return m_radius; }

private:
    float m_radius = 1.f;
};

inline SphereShape::SphereShape(float radius) : m_radius(radius) {}

inline Vec3 SphereShape::support_point(const Vec3& axis, float bloat) const
{
    // TODO: consider getting rid of normalization
    return m_transform.translation() + axis.normalized() * (m_radius + bloat);
}

inline void SphereShape::set_transform(const Mat44& matrix)
{
    m_transform = matrix;
    Vec3 delta = {m_radius, m_radius, m_radius};
    m_aabb.reset(matrix.translation() - delta, matrix.translation() + delta);
}

} // slope
