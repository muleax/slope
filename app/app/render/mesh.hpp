#pragma once
#include "slope/math/vector3.hpp"
#include "slope/math/vector2.hpp"
#include "slope/containers/vector.hpp"
#include "app/system/resource_manager.hpp"

namespace slope {

struct Vertex {
    slope::Vec3 position;
    slope::Vec3 normal;
    slope::Vec2 tex_coords;
};

struct Face {
    uint32_t vertex;
    uint32_t normal;
    uint32_t tex_coords;
};

class MeshResource : public IResource {
public:
    bool load(const String& path) override;

    Vector<slope::Vec3> m_vertices;
    Vector<slope::Vec3> m_normals;
    Vector<slope::Vec2> m_tex_coords;
    Vector<Face>        m_faces;

private:

};

class Mesh {
public:

};

} // slope