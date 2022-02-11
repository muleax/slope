#pragma once
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"
#include "slope/core/stats_holder.hpp"
#include <optional>

namespace slope {

struct SATStats {
    uint64_t cum_test_count = 0;
    uint64_t cum_projection_count = 0;

    void reset()
    {
        *this = {};
    }

    void merge(const SATStats& other)
    {
        cum_test_count +=       other.cum_test_count;
        cum_projection_count += other.cum_projection_count;
    }
};

class SATSolver : public StatsHolder<SATStats> {
public:

    std::optional<vec3> find_penetration_axis(const PolyhedronShape* shape1, const PolyhedronShape* shape2);
    std::optional<vec3> find_penetration_axis(const PolyhedronShape* shape1, const BoxShape* shape2);
    std::optional<vec3> find_penetration_axis(const BoxShape* shape1, const BoxShape* shape2);

private:
    template <class Shape1, class Shape2>
    std::optional<vec3> find_penetration_axis_impl(const Shape1* shape1, const Shape2* shape2);
};

} // slope
