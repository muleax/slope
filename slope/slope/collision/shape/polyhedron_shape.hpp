#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/geometry.hpp"
#include "slope/containers/vector.hpp"
#include <memory>

namespace slope {

// Convex Polyhedron
class PolyhedronShape : public CollisionShapeImpl<ShapeType::Polyhedron> {
public:
    explicit PolyhedronShape(std::shared_ptr<ConvexPolyhedron> geometry);

    void        set_transform(const Mat44& matrix) final;
    Vec3        support_point(const Vec3& axis, float bloat) const final;

    float       get_support_face(const Vec3& axis, Vector<Vec3>& out_support, Vec3& out_face_normal) const;
    Interval    project(const Vec3& axis) const;
    const auto& principal_face_axes() const { return m_principal_face_axes; };
    const auto& principal_edge_axes() const { return m_principal_edge_axes; };

    Vec3        world_face_normal(const ConvexPolyhedron::Face& face) const;

private:
    std::shared_ptr<ConvexPolyhedron> m_geometry;
    Vector<Vec3> m_world_vertices;
    Vector<Vec3> m_principal_face_axes;
    Vector<Vec3> m_principal_edge_axes;
};

inline Vec3 PolyhedronShape::world_face_normal(const ConvexPolyhedron::Face& face) const {
    return m_principal_face_axes[face.normal] * face.normal_direction;
}

inline Interval PolyhedronShape::project(const Vec3& axis) const {
    SL_ASSERT(m_world_vertices.size());

    // TODO: optimize with hill climbing
    Interval itv(axis.dot(m_world_vertices[0]));
    for(auto vit = m_world_vertices.begin() + 1; vit != m_world_vertices.end(); ++vit)
        itv.extend(axis.dot(*vit));

    return itv;
}

} // slope
