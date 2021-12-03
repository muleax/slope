#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/geometry.hpp"
#include "slope/containers/vector.hpp"
#include <memory>

namespace slope {

class ConvexPolyhedronShape : public CollisionShapeImpl<ShapeType::ConvexPolyhedron> {
public:
    explicit ConvexPolyhedronShape(std::shared_ptr<ConvexPolyhedron> geometry);

    void                set_transform(const Mat44& matrix) final;

    Vec3                support_point(const Vec3& axis, float bloat) const final;
    float               get_support_face(const Vec3& axis, Vector<Vec3>& out_support, Vec3& out_face_normal) const;
    Interval            project(const Vec3& axis) const;
    const Vector<Vec3>& unique_world_face_normals() const { return m_world_face_normals; };
    const Vector<Vec3>& unique_world_edge_dirs() const { return m_world_edge_dirs; };
    Vec3                world_face_normal(const ConvexPolyhedron::Face& face) const;

private:
    std::shared_ptr<ConvexPolyhedron> m_geometry;
    Vector<Vec3> m_world_vertices;
    Vector<Vec3> m_world_face_normals;
    Vector<Vec3> m_world_edge_dirs;
};

inline Vec3 ConvexPolyhedronShape::world_face_normal(const ConvexPolyhedron::Face& face) const {
    return m_world_face_normals[face.normal] * face.normal_direction;
}

inline Interval ConvexPolyhedronShape::project(const Vec3& axis) const {
    SL_ASSERT(m_world_vertices.size());

    Interval itv(axis.dot(m_world_vertices[0]));
    for(auto vit = m_world_vertices.begin() + 1; vit != m_world_vertices.end(); ++vit)
        itv.extend(axis.dot(*vit));

    return itv;
}

} // slope
