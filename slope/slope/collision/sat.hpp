#pragma once
#include "slope/collision/shape/convex_polyhedron_shape.hpp"
#include <optional>

namespace slope {

class SATSolver {
public:
    std::optional<Vec3> find_penetration_axis(const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);
};

} // slope
