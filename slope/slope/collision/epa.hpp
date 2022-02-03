#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/gjk.hpp"
#include "slope/containers/vector.hpp"
#include <optional>

namespace slope {

class EPASolver {
public:
    using Simplex = GJKSolver::Simplex;

    struct Config {
        uint32_t max_iteration_count = 80;
        float support_bloat = 0.f;
        float early_threshold = 1e-6f;
        float final_threshold = 0.01f;
    };

    struct Stats {
        uint64_t total_fail_count = 0;
        uint64_t cum_test_count = 0;
        uint64_t cum_iterations_count = 0;
        uint32_t max_iteration_count = 0;
    };

    std::optional<vec3> find_penetration_axis(const CollisionShape* shape1, const CollisionShape* shape2, const Simplex& simplex);

    Config&             config() { return m_config; }
    const Stats&        stats() const { return m_stats; }
    void                reset_stats();

private:
    struct Face {
        uint32_t a;
        uint32_t b;
        uint32_t c;
        vec3 normal;
        float dist = FLOAT_MAX;
        bool obsolete = false;

        // min-heap
        bool operator<(const Face& other) const { return dist > other.dist; }
    };

    void add_face(uint32_t a_idx, uint32_t b_idx, uint32_t c_idx);

    void collect_stats(uint32_t iteration_count, bool fail);

    Vector<vec3>        m_points;
    Vector<Face>        m_heap;
    Vector<uint64_t>    m_edges;
    vec3                m_inner_point;

    Config              m_config;
    Stats               m_stats;
};

} // slope
