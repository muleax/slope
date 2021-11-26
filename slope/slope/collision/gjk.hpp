#pragma once
#include "slope/collision/convex_polyhedron_shape.hpp"
#include "slope/containers/array.hpp"
#include "slope/containers/vector.hpp"
#include <optional>

namespace slope {

class GJKCollider {
public:
    using Simplex = Array<Vec3, 4>;

    bool            collide(const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);
    const Simplex&  simplex() const { return m_simplex; }

private:
    Vec3 update_point(Vec3 a);
    Vec3 update_line(Vec3 b, Vec3 a);
    Vec3 update_triangle(Vec3 c, Vec3 b, Vec3 a);
    std::optional<Vec3> update_tetrahedron(Vec3 d, Vec3 c, Vec3 b, Vec3 a);

    Simplex m_simplex;
    int m_simplex_size = 0;
};

class EPA {
public:
    Vec3 find_penetration_axis(
        const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2, const GJKCollider::Simplex& simplex);

private:
    struct Face {
        uint32_t a;
        uint32_t b;
        uint32_t c;
        Vec3 normal;
        float dist = FLOAT_MAX;
        bool obsolete = false;

        // min-heap
        bool operator<(const Face& other) const { return dist > other.dist; }
    };

    void add_face(uint32_t a_idx, uint32_t b_idx, uint32_t c_idx);

    Vector<Vec3> m_points;
    Vector<Face> m_heap;
    Vector<uint64_t> m_edges_linear;

    Vec3 m_inner_point;
};

} // slope
