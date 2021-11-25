#pragma once
#include "slope/collision/convex_polyhedron_shape.hpp"
#include <optional>

namespace slope {

class GJKCollider {
public:
    bool collide(const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);

private:
    Vec3 support(const Vec3& axis) const;

    Vec3 update_point(Vec3 a);
    Vec3 update_line(Vec3 b, Vec3 a);
    Vec3 update_triangle(Vec3 c, Vec3 b, Vec3 a);
    std::optional<Vec3> update_tetrahedron(Vec3 d, Vec3 c, Vec3 b, Vec3 a);

    Vec3 m_simplex[4];
    int m_simplex_size = 0;
    const ConvexPolyhedronShape* m_shape1 = nullptr;
    const ConvexPolyhedronShape* m_shape2 = nullptr;
};

} // slope
