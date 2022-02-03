#pragma once
#include "slope/math/vector2.hpp"
#include "slope/math/vector3.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/array.hpp"
#include <memory>

namespace slope {

class ConvexPolyhedron;

class Trimesh {
public:
    struct Triangle {
        Array<uint32_t, 3>  vert_ids;
        uint32_t            normal_id;
    };

    const Vector<vec3>&     vertices() const { return m_vertices; }
    const Vector<vec3>&     normals() const { return m_normals; }
    const Vector<Triangle>& triangles() const { return m_triangles; }

private:
    Vector<vec3>        m_vertices;
    Vector<vec3>        m_normals;
    Vector<Triangle>    m_triangles;

    friend class TrimeshFactory;
};

class TrimeshFactory {
public:
    using GeomPtr = std::shared_ptr<Trimesh>;

    void        clear();
    uint32_t    add_vertex(const vec3& vertex);
    uint32_t    add_normal(const vec3& normal);
    uint32_t    add_tri(const Trimesh::Triangle& tri);

    GeomPtr     build() const { return std::make_shared<Trimesh>(m_result); }

    GeomPtr     from_polyhedron(const ConvexPolyhedron& poly);

private:
    Trimesh m_result;
};

class ConvexPolyhedron {
public:
    struct Face {
        uint32_t    first_vertex;
        uint32_t    vertex_count;
        uint32_t    normal;
        float       normal_direction;
    };

    const Vector<vec3>& vertices() const { return m_vertices; }
    const Vector<Face>& faces() const { return m_faces; }
    vec3                face_normal(const Face& face) const;
    const Vector<vec3>& edge_dirs() const { return m_edge_dirs; }
    const Vector<vec3>& face_normals() const { return m_face_normals; }
    uint32_t            vertex_index(const Face& face, uint32_t vert_id) const;

private:
    Vector<vec3>        m_vertices;
    Vector<vec3>        m_face_normals;
    Vector<vec3>        m_edge_dirs;
    Vector<uint32_t>    m_face_indices;
    Vector<Face>        m_faces;

    friend class ConvexPolyhedronFactory;
};

class ConvexPolyhedronFactory {
public:
    using GeomPtr = std::shared_ptr<ConvexPolyhedron>;

    void        clear();
    uint32_t    add_vertex(const vec3& vertex);
    uint32_t    add_face(VectorView<uint32_t> indices);

    GeomPtr     build() { return std::make_shared<ConvexPolyhedron>(m_result); }

    GeomPtr     convex_hull(const Vector<vec3>& vertices);
    GeomPtr     box(vec3 dimensions, vec3 offset = {});

private:
    ConvexPolyhedron m_result;
};

inline vec3 ConvexPolyhedron::face_normal(const Face& face) const {
    return m_face_normals[face.normal] * face.normal_direction;
}

inline uint32_t ConvexPolyhedron::vertex_index(const Face& face, uint32_t vert_id) const {
    return m_face_indices[face.first_vertex + vert_id];
}

} // slope