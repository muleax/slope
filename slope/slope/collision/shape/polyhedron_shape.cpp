#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

PolyhedronShape::PolyhedronShape(std::shared_ptr<ConvexPolyhedron> geometry)
    : m_geometry(std::move(geometry))
{
    auto& geom = *m_geometry;
    m_world_vertices.reserve(geom.vertices().size());
    m_principal_face_axes.reserve(geom.face_normals().size());
    m_principal_edge_axes.reserve(geom.edge_dirs().size());
}

void PolyhedronShape::set_transform(const Mat44& matrix)
{
    m_transform = matrix;
    auto& geom = *m_geometry;

    m_world_vertices.clear();
    for (auto& vert: geom.vertices()) {
        m_world_vertices.push_back(m_transform.apply_point(vert));
    }

    m_principal_face_axes.clear();
    for (auto& normal: geom.face_normals()) {
        m_principal_face_axes.push_back(m_transform.apply_normal(normal));
    }

    m_principal_edge_axes.clear();
    for (auto& dir: geom.edge_dirs()) {
        m_principal_edge_axes.push_back(m_transform.apply_normal(dir));
    }

    m_aabb.reset(m_world_vertices[0]);
    for (auto vit = m_world_vertices.begin() + 1; vit != m_world_vertices.end(); ++vit) {
        m_aabb.extend(*vit);
    }
}

Vec3 PolyhedronShape::support_point(const Vec3& axis, float bloat) const
{
    float max_dot = -FLOAT_MAX;
    const Vec3* best_point = nullptr;
    for (auto& p : m_world_vertices) {
        float dot = axis.dot(p);
        if (dot > max_dot) {
            max_dot = dot;
            best_point = &p;
        }
    }

    return *best_point;
}

float PolyhedronShape::get_support_face(const Vec3& axis, Vector<Vec3>& out_support, Vec3& out_face_normal) const
{
    auto& geom = *m_geometry;

    float max_dot = -FLOAT_MAX;
    const ConvexPolyhedron::Face* best_face = nullptr;

    for(auto& face : geom.faces()) {
        float dot = axis.dot(world_face_normal(face));
        if(dot > max_dot) {
            max_dot = dot;
            best_face = &face;
        }
    }

    out_face_normal = world_face_normal(*best_face);
    out_support.clear();
    for(uint32_t i = 0; i < best_face->vertex_count; i++) {
        out_support.push_back(m_world_vertices[geom.vertex_index(*best_face, i)]);
    }

    return max_dot * max_dot;
}

} // slope