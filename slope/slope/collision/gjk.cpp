#include "slope/collision/gjk.hpp"
#include "slope/collision/primitives.hpp"
#include <optional>

namespace slope {

void GJKSolver::reset_stats()
{
    m_stats.cum_test_count = 0;
    m_stats.cum_iterations_count = 0;
    m_stats.max_iteration_count = 0;
}

void GJKSolver::collect_stats(uint32_t iteration_count, bool fail)
{
    m_stats.cum_test_count++;
    m_stats.cum_iterations_count += iteration_count;
    m_stats.max_iteration_count = std::max(m_stats.max_iteration_count, iteration_count);
    m_stats.total_fail_count += (int)fail;
}

inline vec3 GJKSolver::update_point(vec3 a)
{
    m_simplex_size = 1;
    m_simplex[0] = a;

    vec3 axis = a.is_zero() ? vec3{1.f, 0.f, 0.f} : -a;
    return axis;
}

inline vec3 GJKSolver::update_line(vec3 b, vec3 a)
{
    m_simplex_size = 2;
    m_simplex[0] = b;
    m_simplex[1] = a;

    vec3 ab = b - a;
    vec3 ao = -a;

    vec3 axis = ab.cross(ao).cross(ab);
    if (axis.is_zero())
        axis = find_orthogonal(ao);

    return axis;
}

vec3 GJKSolver::update_triangle(vec3 c, vec3 b, vec3 a)
{
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 ao = -a;
    vec3 n = ab.cross(ac);

    if (ab.cross(n).dot(ao) > 0.f) {
        return update_line(b, a);
    }

    if (n.cross(ac).dot(ao) > 0.f) {
        return update_line(c, a);
    }

    m_simplex_size = 3;
    m_simplex[0] = c;
    m_simplex[1] = b;
    m_simplex[2] = a;

    return n.dot(ao) >= 0.f ? n : -n;
}

std::optional<vec3> GJKSolver::update_tetrahedron(vec3 d, vec3 c, vec3 b, vec3 a)
{
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 ad = d - a;
    vec3 ao = -a;

    vec3 abc = ab.cross(ac);
    if (abc.dot(ad) * abc.dot(ao) < 0.f) {
        return update_triangle(c, b, a);
    }

    vec3 abd = ab.cross(ad);
    if (abd.dot(ac) * abd.dot(ao) < 0.f) {
        return update_triangle(d, b, a);
    }

    vec3 acd = ac.cross(ad);
    if (acd.dot(ab) * acd.dot(ao) < 0.f) {
        return update_triangle(d, c, a);
    }

    return std::nullopt;
}

bool GJKSolver::intersect(const CollisionShape* shape1, const CollisionShape* shape2)
{
    constexpr float SUPPORT_EPSILON = 1e-12f;

    vec3 init_axis = shape1->transform().translation() - shape2->transform().translation();
    m_simplex[0] = shape1->support_diff(shape2, init_axis, 0.f, false);
    m_simplex_size = 1;

    for (uint32_t iter = 0; iter < m_config.max_iteration_count; iter++) {
        std::optional<vec3> axis;

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
            //slope::log::info("GJK Collision iter {}", iter);
            SL_ASSERT(m_simplex_size == 4);
            collect_stats(iter + 1, false);
            return true;
        }

        vec3 new_pt = shape1->support_diff(shape2, *axis, 0.f, false);

        float proximity = axis->dot(new_pt);
        if (proximity < SUPPORT_EPSILON) {
            collect_stats(iter + 1, false);
            return false;
        }

        m_simplex[m_simplex_size++] = new_pt;
    }

    collect_stats(m_config.max_iteration_count, true);
    return false;
}

} // slope
