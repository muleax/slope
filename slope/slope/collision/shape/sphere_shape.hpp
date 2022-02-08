#pragma once
#include "slope/collision/shape/collision_shape.hpp"

namespace slope {

class SphereShape : public TypedObject<CollisionShape, ShapeKind::Sphere> {
public:
    explicit SphereShape(float radius);

    void    set_transform(const mat44& matrix) final;
    vec3    support(const vec3& axis, float bloat, bool normalized) const final;

    float   radius() const { return m_radius; }

private:
    float m_radius = 1.f;
};

inline SphereShape::SphereShape(float radius) : m_radius(radius) {}

inline vec3 SphereShape::support(const vec3& axis, float bloat, bool normalized) const
{
    vec3 norm_axis = normalized ? axis : axis.normalized();
    return m_transform.translation() + norm_axis * (m_radius + bloat);
}

inline void SphereShape::set_transform(const mat44& matrix)
{
    m_transform = matrix;
    vec3 delta = {m_radius, m_radius, m_radius};
    m_aabb.reset(matrix.translation() - delta, matrix.translation() + delta);
}

} // slope
