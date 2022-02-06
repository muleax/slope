#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/core/array.hpp"
#include <optional>

namespace slope {

class GJKSolver {
public:
    using Simplex = Array<vec3, 4>;

    struct Config {
        uint32_t max_iteration_count = 30;
    };

    struct Stats {
        uint64_t total_fail_count = 0;
        uint64_t cum_test_count = 0;
        uint64_t cum_iterations_count = 0;
        uint32_t max_iteration_count = 0;
    };

    bool            intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    const Simplex&  simplex() const { return m_simplex; }

    Config&         config() { return m_config; }
    const Stats&    stats() const { return m_stats; }
    void            reset_stats();

private:
    vec3 update_point(vec3 a);
    vec3 update_line(vec3 b, vec3 a);
    vec3 update_triangle(vec3 c, vec3 b, vec3 a);
    std::optional<vec3> update_tetrahedron(vec3 d, vec3 c, vec3 b, vec3 a);

    void collect_stats(uint32_t iteration_count, bool fail);

    Simplex     m_simplex;
    uint32_t    m_simplex_size = 0;

    Config      m_config;
    Stats       m_stats;
};

} // slope
