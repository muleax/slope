#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/core/array.hpp"
#include "slope/core/stats_holder.hpp"
#include "slope/core/config_holder.hpp"
#include <optional>

namespace slope {

struct GJKConfig {
    uint32_t max_iteration_count = 30;
};

struct GJKStats {
    uint64_t total_fail_count = 0;
    uint64_t cum_test_count = 0;
    uint64_t cum_iterations_count = 0;
    uint32_t max_iteration_count = 0;

    void reset()
    {
        cum_test_count = 0;
        cum_iterations_count = 0;
        max_iteration_count = 0;
    }

    void merge(const GJKStats& other)
    {
        total_fail_count +=     other.total_fail_count;
        cum_test_count +=       other.cum_test_count;
        cum_iterations_count += other.cum_iterations_count;
        max_iteration_count =   std::max(max_iteration_count, other.max_iteration_count);
    }
};

class GJKSolver : public ConfigHolder<GJKConfig>, public StatsHolder<GJKStats> {
public:
    using Simplex = Array<vec3, 4>;

    bool            intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    const Simplex&  simplex() const { return m_simplex; }

private:
    vec3 update_point(vec3 a);
    vec3 update_line(vec3 b, vec3 a);
    vec3 update_triangle(vec3 c, vec3 b, vec3 a);
    std::optional<vec3> update_tetrahedron(vec3 d, vec3 c, vec3 b, vec3 a);

    void collect_stats(uint32_t iteration_count, bool fail);

    Simplex     m_simplex;
    uint32_t    m_simplex_size = 0;
};

} // slope
