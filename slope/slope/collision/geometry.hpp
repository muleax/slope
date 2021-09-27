#pragma once
#include "slope/math/vector2.hpp"
#include "slope/math/vector3.hpp"
#include "slope/containers/vector.hpp"
#include "nlohmann/json.hpp"

namespace slope {

class Polyhedron {
public:

    uint32_t add_vertex(const slope::Vec3& vertex);
    uint32_t add_face(const Vector<uint32_t>& face_indices);

    bool load(const nlohmann::json& data)
    {
        return false;
    }

private:
    struct Face {
        uint32_t    begin_index;
        uint32_t    size;
        float       normal_direction;
    };

    Vector<Vec3>        m_vertices;
    Vector<Vec3>        m_normals;
    Vector<Vec3>        m_edge_directions;
    Vector<uint32_t>    m_face_indices;
};

} // slope