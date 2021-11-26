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

std::optional<Vec3> SATSolver::find_penetration_axis(
    const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2)
{
    const float EDGE_CROSS_EPSILON = 1e-12f;

    IntervalPenetration min_pen;
    Vec3 min_pen_axis;

    auto sat_test = [&min_pen_axis, &min_pen, shape1, shape2](const Vec3& axis) -> bool {
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

    for (auto& normal : shape1->unique_world_face_normals()) {
        if (!sat_test(normal))
            return std::nullopt;
    }

    for (auto& normal : shape2->unique_world_face_normals()) {
        if (!sat_test(normal))
            return std::nullopt;
    }

    for (auto& edge1 : shape1->unique_world_edge_dirs()) {
        for(auto& edge2 : shape2->unique_world_edge_dirs()) {
            auto axis = edge1.cross(edge2);
            float axis_sqr_len = axis.length_squared();
            if(axis_sqr_len < EDGE_CROSS_EPSILON)
                continue;

            axis /= sqrtf(axis_sqr_len);

            if (!sat_test(axis))
                return std::nullopt;
        }
    }

    return min_pen_axis * min_pen.direction;
}

} // slope
