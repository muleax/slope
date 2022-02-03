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

void PolyhedronShape::set_transform(const mat44& matrix)
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

float PolyhedronShape::get_support_face(const vec3& axis, Vector<vec3>& out_support, vec3& out_face_normal) const
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
