#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/core/array.hpp"
#include "slope/core/vector.hpp"
#include <memory>

namespace slope {

class BoxShape : public TypedObject<CollisionShape, ShapeKind::Box> {
public:
    explicit BoxShape(const vec3& dimensions);

    void        set_transform(const mat44& matrix) final;
    vec3        support(const vec3& axis, float bloat, bool normalized) const final;

    // Polyhedra-specific interface
    float       get_support_face(const vec3& axis, Vector<vec3>& out_support, vec3& out_face_normal) const;
    Interval    project(const vec3& axis) const;
    const auto& principal_face_axes() const { return m_principal_axes; };
    const auto& principal_edge_axes() const { return m_principal_axes; };

private:
    vec3 m_half_dimensions;
    Array<vec3, 3> m_extents;
    Array<vec3, 3> m_principal_axes;
};

inline Interval BoxShape::project(const vec3& axis) const
{
    float dot = 0.f;
    for (int i = 0; i < 3; i++)
        dot += fabs(axis.dot(m_extents[i]));

    auto pdot = axis.dot(m_transform.translation());
    return { pdot - dot, pdot + dot };
}

inline vec3 BoxShape::support(const vec3& axis, float bloat, bool normalized) const
{
    vec3 offs;
    for (int i = 0; i < 3; i++) {
        float dot = axis.dot(m_extents[i]);
        offs += m_extents[i] * signf(dot);
    }

    return m_transform.translation() + offs;
}

} // slope
