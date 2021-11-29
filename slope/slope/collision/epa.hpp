#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/gjk.hpp"
#include "slope/containers/vector.hpp"
#include <optional>

namespace slope {

class EPASolver {
public:
    std::optional<Vec3> find_penetration_axis(
        const CollisionShape* shape1, const CollisionShape* shape2, const GJKSolver::Simplex& simplex);

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
    Vector<uint64_t> m_edges;

    Vec3 m_inner_point;
};

} // slope
