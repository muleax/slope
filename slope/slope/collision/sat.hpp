#pragma once
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"
#include <optional>

namespace slope {

class SATSolver {
public:
    struct Stats {
        uint64_t cum_test_count = 0;
        uint64_t cum_projection_count = 0;
    };

    std::optional<Vec3> find_penetration_axis(const PolyhedronShape* shape1, const PolyhedronShape* shape2);
    std::optional<Vec3> find_penetration_axis(const PolyhedronShape* shape1, const BoxShape* shape2);
    std::optional<Vec3> find_penetration_axis(const BoxShape* shape1, const BoxShape* shape2);

    const Stats&        stats() const { return m_stats; }
    void                reset_stats();

private:
    Stats m_stats;
};

} // slope
