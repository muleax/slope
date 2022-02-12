#pragma once
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"
#include <optional>

namespace slope {

struct SATStats {
    uint64_t cum_test_count = 0;
    uint64_t cum_projection_count = 0;

    void reset();
    void merge(const SATStats& other);
};

class SATSolver {
public:

    std::optional<vec3> find_penetration_axis(const PolyhedronShape* shape1, const PolyhedronShape* shape2);
    std::optional<vec3> find_penetration_axis(const PolyhedronShape* shape1, const BoxShape* shape2);
    std::optional<vec3> find_penetration_axis(const BoxShape* shape1, const BoxShape* shape2);

    void                reset_stats() { m_stats.reset(); }
    const auto&         stats() const { return m_stats; }

private:
    template <class Shape1, class Shape2>
    std::optional<vec3> find_penetration_axis_impl(const Shape1* shape1, const Shape2* shape2);

    SATStats m_stats;
};

} // slope
