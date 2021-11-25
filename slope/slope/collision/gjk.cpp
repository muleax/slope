#include "slope/collision/gjk.hpp"
#include "slope/debug/log.hpp"

namespace slope {

inline Vec3 GJKCollider::support(const Vec3& axis) const {
    return m_shape1->support_point(axis) - m_shape2->support_point(-axis);
}

Vec3 GJKCollider::update_point(Vec3 a) {
    m_simplex_size = 1;
    m_simplex[0] = a;
    return -a;
}

Vec3 GJKCollider::update_line(Vec3 b, Vec3 a) {
    m_simplex_size = 2;
    m_simplex[0] = b;
    m_simplex[1] = a;

    Vec3 ab = b - a;
    Vec3 ao = -a;

    return ab.cross(ao).cross(ab);
}

Vec3 GJKCollider::update_triangle(Vec3 c, Vec3 b, Vec3 a) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ao = -a;
    Vec3 n = ab.cross(ac);

    if (ab.cross(n).dot(ao) >= 0.f) {
        return update_line(b, a);
    }

    if (n.cross(ac).dot(ao) >= 0.f) {
        return update_line(c, a);
    }

    m_simplex_size = 3;
    m_simplex[0] = c;
    m_simplex[1] = b;
    m_simplex[2] = a;

    return n.dot(ao) >= 0.f ? n : -n;
}

std::optional<Vec3> GJKCollider::update_tetrahedron(Vec3 d, Vec3 c, Vec3 b, Vec3 a) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ad = d - a;
    Vec3 ao = -a;

    Vec3 abc = ab.cross(ac);
    if (abc.dot(ad) * abc.dot(ao) <= 0.f) {
        return update_triangle(c, b, a);
    }

    Vec3 abd = ab.cross(ad);
    if (abd.dot(ac) * abd.dot(ao) <= 0.f) {
        return update_triangle(d, b, a);
    }

    Vec3 acd = ac.cross(ad);
    if (acd.dot(ab) * acd.dot(ao) <= 0.f) {
        return update_triangle(d, c, a);
    }

    return std::nullopt;
}

bool GJKCollider::collide(const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2) {
    constexpr float SUPPORT_EPSILON = 1e-12f;

    m_shape1 = shape1;
    m_shape2 = shape2;

    Vec3 init_axis = shape1->transform().translation() - shape2->transform().translation();
    m_simplex[0] = support(init_axis);
    m_simplex_size = 1;

    for (int iter = 0; iter < 50; iter++) {
        std::optional<Vec3> axis;

        switch (m_simplex_size) {
        case 1: {
            axis = update_point(m_simplex[0]);
            break;
        }
        case 2: {
            axis = update_line(m_simplex[0], m_simplex[1]);
            break;
        }
        case 3: {
            axis = update_triangle(m_simplex[0], m_simplex[1], m_simplex[2]);
            break;
        }
        case 4: {
            axis = update_tetrahedron(m_simplex[0], m_simplex[1], m_simplex[2], m_simplex[3]);
            break;
        }
        default: {
            SL_VERIFY(false);
            break;
        }
        }

        if (!axis) {
            slope::log::info("GJK Collision iter {}", iter);
            return true;
        }

        Vec3 new_pt = support(*axis);

        if (axis->dot(new_pt) < SUPPORT_EPSILON)
            return false;

        m_simplex[m_simplex_size++] = new_pt;
    }

    return false;
}

} // slope
