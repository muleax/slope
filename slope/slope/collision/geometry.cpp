#include "slope/collision/geometry.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

uint32_t TrimeshFactory::add_vertex(const vec3& vertex) {
    m_result.m_vertices.push_back(vertex);
    return m_result.m_vertices.size() - 1;
}

uint32_t TrimeshFactory::add_normal(const vec3& normal) {
    m_result.m_normals.push_back(normal);
    return m_result.m_normals.size() - 1;

}

uint32_t TrimeshFactory::add_tri(const Trimesh::Triangle& tri) {
    m_result.m_triangles.push_back(tri);
    return m_result.m_triangles.size() - 1;
}

void TrimeshFactory::clear() {
    m_result.m_vertices.clear();
    m_result.m_normals.clear();
    m_result.m_triangles.clear();
}

TrimeshFactory::GeomPtr TrimeshFactory::from_polyhedron(const ConvexPolyhedron& poly) {
    clear();

    m_result.m_vertices = poly.vertices();

    for (auto& face : poly.faces()) {
        auto normal_id = static_cast<uint32_t>(m_result.m_normals.size());
        m_result.m_normals.push_back(poly.face_normal(face));

        for (uint32_t i = 2; i < face.vertex_count; i++) {
            auto& tri = m_result.m_triangles.emplace_back();
            tri.vert_ids = {poly.vertex_index(face, 0), poly.vertex_index(face, i-1), poly.vertex_index(face, i)};
            tri.normal_id = normal_id;
        }
    }

    return build();
}

uint32_t ConvexPolyhedronFactory::add_vertex(const vec3& vertex) {
    m_result.m_vertices.push_back(vertex);
    return static_cast<uint32_t>(m_result.m_vertices.size() - 1);
}

uint32_t ConvexPolyhedronFactory::add_face(VectorView<uint32_t> indices) {
    SL_ASSERT(indicies.size() > 2);

    auto& face = m_result.m_faces.emplace_back();
    face.first_vertex = static_cast<uint32_t>(m_result.m_face_indices.size());
    face.vertex_count = static_cast<uint32_t>(indices.size());

    m_result.m_face_indices.insert(m_result.m_face_indices.end(), indices.begin(), indices.end());

    const vec3& a = m_result.m_vertices[indices[0]];
    const vec3& b = m_result.m_vertices[indices[1]];
    const vec3& c = m_result.m_vertices[indices[1]];
    vec3 normal = (b - a).cross(c - a);
    SL_ASSERT(normal.length_squared() > 1e-12f);
    normal = normal.normalized();

    // TODO: optimize
    auto find_equal_dir = [](const vec3& dir, Vector<vec3>& search_vector) -> uint32_t {
        for (size_t i = 0; i < search_vector.size(); i++) {
            if (search_vector[i].cross(dir).length_squared() < 1e-12f) {
                return i;
            }
        }

        return search_vector.size();
    };

    face.normal = find_equal_dir(normal, m_result.m_face_normals);

    if (face.normal < m_result.m_face_normals.size()) {
        face.normal_direction = m_result.m_face_normals[face.normal].dot(normal) < 0.f ? -1.f : 1.f;
    } else {
        m_result.m_face_normals.push_back(normal);
        face.normal_direction = 1.f;
    }

    for (size_t i = 1; i < indices.size(); i++) {
        auto edge = (m_result.m_vertices[i] - m_result.m_vertices[i - 1]).normalized();

        if (find_equal_dir(edge, m_result.m_edge_dirs) == m_result.m_edge_dirs.size()) {
            m_result.m_edge_dirs.push_back(edge);
        }
    }

    return m_result.m_faces.size() - 1;
}

void ConvexPolyhedronFactory::clear() {
    m_result.m_vertices.clear();
    m_result.m_face_normals.clear();
    m_result.m_edge_dirs.clear();
    m_result.m_face_indices.clear();
    m_result.m_faces.clear();
}

ConvexPolyhedronFactory::GeomPtr ConvexPolyhedronFactory::convex_hull(const Vector<vec3>& vertices) {
    // TODO: quickhull
    return nullptr;
}

ConvexPolyhedronFactory::GeomPtr ConvexPolyhedronFactory::box(vec3 dimensions, vec3 offset) {
    clear();

    float extent[] = {-0.5f, 0.5f};

    m_result.m_vertices.reserve(8);

    for (float ex : extent) {
        float x = ex * dimensions.x + offset.x;
        for (float ey : extent) {
            float y = ey * dimensions.y + offset.y;
            for (float ez : extent) {
                float z = ez * dimensions.z + offset.z;
                m_result.m_vertices.emplace_back(x, y, z);
            }
        }
    }

    m_result.m_face_normals.reserve(3);
    m_result.m_edge_dirs.reserve(3);

    for (int i = 0; i < 3; i++) {
        m_result.m_face_normals.emplace_back().data[i] = 1.f;
        m_result.m_edge_dirs.emplace_back().data[i] = 1.f;
    }

    m_result.m_face_indices.reserve(4 * 6);
    m_result.m_faces.reserve(6);

    auto _add_face = [this](uint32_t normal, float normal_dir, const Array<uint32_t, 4>& verts) {
        auto first_vertex = static_cast<uint32_t>(m_result.m_face_indices.size());
        m_result.m_face_indices.insert(m_result.m_face_indices.end(), verts.begin(), verts.end());
        m_result.m_faces.push_back({first_vertex, 4, normal, normal_dir});
    };

    _add_face(0, 1.f, {7, 6, 4, 5});
    _add_face(0, -1.f, {2, 3, 1, 0});

    _add_face(1, 1.f, {7, 3, 2, 6});
    _add_face(1, -1.f, {1, 5, 4, 0});

    _add_face(2, 1.f, {3, 7, 5, 1});
    _add_face(2, -1.f, {6, 2, 0, 4});

    return build();
}

} // slope