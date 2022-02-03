#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/geometry.hpp"
#include "slope/containers/vector.hpp"
#include <memory>

namespace slope {

// Convex Polyhedron
class PolyhedronShape : public TypedCollisionShape<ShapeKind::Polyhedron> {
public:
    explicit PolyhedronShape(std::shared_ptr<ConvexPolyhedron> geometry);

    void        set_transform(const mat44& matrix) final;
    vec3        support(const vec3& axis, float bloat, bool normalized) const final;

    // Polyhedra-specific interface
    float       get_support_face(const vec3& axis, Vector<vec3>& out_support, vec3& out_face_normal) const;
    Interval    project(const vec3& axis) const;
    const auto& principal_face_axes() const { return m_principal_face_axes; };
    const auto& principal_edge_axes() const { return m_principal_edge_axes; };

    vec3        world_face_normal(const ConvexPolyhedron::Face& face) const;

private:
    std::shared_ptr<ConvexPolyhedron> m_geometry;
    Vector<vec3> m_world_vertices;
    Vector<vec3> m_principal_face_axes;
    Vector<vec3> m_principal_edge_axes;
};

inline vec3 PolyhedronShape::world_face_normal(const ConvexPolyhedron::Face& face) const {
    return m_principal_face_axes[face.normal] * face.normal_direction;
}

inline Interval PolyhedronShape::project(const vec3& axis) const {
    SL_ASSERT(m_world_vertices.size());

    // TODO: optimize with hill climbing
    Interval itv(axis.dot(m_world_vertices[0]));
    for(auto vit = m_world_vertices.begin() + 1; vit != m_world_vertices.end(); ++vit)
        itv.extend(axis.dot(*vit));

    return itv;
}

inline vec3 PolyhedronShape::support(const vec3& axis, float bloat, bool normalized) const
{
    // TODO: optimize with hill climbing
    float max_dot = -FLOAT_MAX;
    const vec3* best_point = nullptr;
    for (auto& p : m_world_vertices) {
        float dot = axis.dot(p);
        if (dot > max_dot) {
            max_dot = dot;
            best_point = &p;
        }
    }

    return *best_point;
}

} // slope
