#pragma once
#include "slope/math/vector2.hpp"
#include "slope/math/vector3.hpp"
#include "slope/containers/vector.hpp"
#include "slope/containers/array.hpp"
#include "nlohmann/json_fwd.hpp"

namespace slope {

class ConvexPolyhedron;

class Trimesh {
public:
    struct Triangle {
        Array<uint32_t, 3>  vert_ids;
        uint32_t            normal_id;
    };

    const Vector<Vec3>&     vertices() const { return m_vertices; }
    const Vector<Vec3>&     normals() const { return m_normals; }
    const Vector<Triangle>& triangles() const { return m_triangles; }

private:
    Vector<Vec3>        m_vertices;
    Vector<Vec3>        m_normals;
    Vector<Triangle>    m_triangles;

    friend class TrimeshFactory;
};

class TrimeshFactory {
public:
    uint32_t    add_vertex(const Vec3& vertex);
    uint32_t    add_normal(const Vec3& normal);
    uint32_t    add_tri(const Trimesh::Triangle& tri);
    void        clear();

    void        from_polyhedron(const ConvexPolyhedron& poly);

    const Trimesh& result() const { return m_result; }

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

    const Vector<Vec3>& vertices() const { return m_vertices; }
    const Vector<Face>& faces() const { return m_faces; }
    Vec3                face_normal(const Face& face) const;

    uint32_t            vertex_index(const Face& face, uint32_t vert_id) const { return m_face_indices[face.first_vertex + vert_id]; }

private:
    Vector<Vec3>        m_vertices;
    Vector<Vec3>        m_face_normals;
    Vector<Vec3>        m_edge_dirs;
    Vector<uint32_t>    m_face_indices;
    Vector<Face>        m_faces;

    friend class ConvexPolyhedronFactory;
};

class ConvexPolyhedronFactory {
public:
    uint32_t    add_vertex(const Vec3& vertex);
    uint32_t    add_face(VectorView<uint32_t> indices);
    void        clear();

    bool        convex_hull(const Vector<Vec3>& vertices);
    void        box(Vec3 dimensions, Vec3 offset = {});

    void        save(const nlohmann::json& data) const;
    bool        load(const nlohmann::json& data);

    const ConvexPolyhedron& result() const { return m_result; }

private:
    ConvexPolyhedron m_result;
};

inline Vec3 ConvexPolyhedron::face_normal(const Face& face) const {
    return m_face_normals[face.normal] * face.normal_direction;
}

} // slope