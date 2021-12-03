#pragma once
#include "slope/collision/shape/convex_polyhedron_shape.hpp"
#include <optional>

namespace slope {

class SATSolver {
public:
    struct Stats {
        uint64_t cum_test_count = 0;
        uint64_t cum_projection_count = 0;
    };

    std::optional<Vec3> find_penetration_axis(const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);

    const Stats&        stats() const { return m_stats; }
    void                reset_stats();

private:
    Stats m_stats;
};

} // slope
