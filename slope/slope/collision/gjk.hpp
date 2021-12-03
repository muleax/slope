#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/containers/array.hpp"
#include <optional>

namespace slope {

class GJKSolver {
public:
    using Simplex = Array<Vec3, 4>;

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
    Vec3 update_point(Vec3 a);
    Vec3 update_line(Vec3 b, Vec3 a);
    Vec3 update_triangle(Vec3 c, Vec3 b, Vec3 a);
    std::optional<Vec3> update_tetrahedron(Vec3 d, Vec3 c, Vec3 b, Vec3 a);

    void collect_stats(uint32_t iteration_count, bool fail);

    Simplex     m_simplex;
    uint32_t    m_simplex_size = 0;

    Config      m_config;
    Stats       m_stats;
};

} // slope
