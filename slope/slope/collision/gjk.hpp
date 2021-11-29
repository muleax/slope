#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/containers/array.hpp"
#include <optional>

namespace slope {

class GJKSolver {
public:
    using Simplex = Array<Vec3, 4>;

    bool            intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    const Simplex&  simplex() const { return m_simplex; }

private:
    Vec3 update_point(Vec3 a);
    Vec3 update_line(Vec3 b, Vec3 a);
    Vec3 update_triangle(Vec3 c, Vec3 b, Vec3 a);
    std::optional<Vec3> update_tetrahedron(Vec3 d, Vec3 c, Vec3 b, Vec3 a);

    Simplex m_simplex;
    int m_simplex_size = 0;
};

} // slope
