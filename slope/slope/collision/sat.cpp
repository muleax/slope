#include "slope/collision/sat.hpp"
#include "slope/debug/assert.hpp"

namespace slope {

namespace {

struct IntervalPenetration {
    float depth = FLOAT_MAX;
    float direction = 1.f;
};

inline IntervalPenetration get_interval_penetration(const Interval& a, const Interval& b)
{
    SL_ASSERT(a.intersects(b));

    float pen1 = b.max - a.min;
    float pen2 = a.max - b.min;
    if (pen2 < pen1) {
        return {pen2, 1.f};
    } else {
        return {pen1, -1.f};
    }
}

} // unnamed

void SATSolver::reset_stats()
{
    m_stats.cum_test_count = 0;
    m_stats.cum_projection_count = 0;
}

template <class Shape1, class Shape2>
std::optional<vec3> SATSolver::find_penetration_axis_impl(const Shape1* shape1, const Shape2* shape2)
{
    const float EDGE_CROSS_EPSILON = 1e-12f;

    m_stats.cum_test_count++;

    IntervalPenetration min_pen;
    vec3 min_pen_axis;

    auto sat_test = [this, &min_pen_axis, &min_pen, shape1, shape2](const vec3& axis) -> bool {
        m_stats.cum_projection_count++;

        Interval itv1 = shape1->project(axis);
        Interval itv2 = shape2->project(axis);
        if(!itv1.intersects(itv2))
            return false;

        auto pen = get_interval_penetration(itv1, itv2);
        if(pen.depth < min_pen.depth) {
            min_pen_axis = axis;
            min_pen = pen;
        }

        return true;
    };

    for (auto& normal : shape1->principal_face_axes()) {
        if (!sat_test(normal))
            return std::nullopt;
    }

    for (auto& normal : shape2->principal_face_axes()) {
        if (!sat_test(normal))
            return std::nullopt;
    }

    for (auto& edge1 : shape1->principal_edge_axes()) {
        for(auto& edge2 : shape2->principal_edge_axes()) {
            auto axis = edge1.cross(edge2);
            float axis_sqr_len = axis.length_squared();
            if (axis_sqr_len < EDGE_CROSS_EPSILON)
                continue;

            axis /= sqrtf(axis_sqr_len);

            if (!sat_test(axis))
                return std::nullopt;
        }
    }

    return min_pen_axis * min_pen.direction;
}

std::optional<vec3> SATSolver::find_penetration_axis(const PolyhedronShape* shape1, const PolyhedronShape* shape2)
{
    return find_penetration_axis_impl(shape1, shape2);
}

std::optional<vec3> SATSolver::find_penetration_axis(const PolyhedronShape* shape1, const BoxShape* shape2)
{
    return find_penetration_axis_impl(shape1, shape2);
}

std::optional<vec3> SATSolver::find_penetration_axis(const BoxShape* shape1, const BoxShape* shape2)
{
    return find_penetration_axis_impl(shape1, shape2);
}

} // slope
