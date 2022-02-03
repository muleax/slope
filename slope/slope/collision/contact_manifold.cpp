#include "slope/collision/contact_manifold.hpp"

namespace slope {

namespace {

void reset_point(ManifoldPoint& point, const ContactGeom& new_geom, const vec3& local_point, float penetration) {
    point.geom = new_geom;
    point.local_point = local_point;
    point.penetration = penetration;
}

float evaluate_area(vec3 p0, vec3 p1, vec3 p2, vec3 p3) {
    auto v0 = (p0 - p1).cross(p2 - p3);
    auto v1 = (p0 - p2).cross(p1 - p3);
    auto v2 = (p0 - p3).cross(p1 - p2);

    return std::fmax(std::fmax(v0.length_squared(), v1.length_squared()), v2.length_squared());
}

} // unnamed

void ContactManifold::begin_update() {
    m_lambda_cache_size = 0;
    for (auto& pt: *this) {
        auto& cache = m_lambda_cache[m_lambda_cache_size++];
        cache.local_point = pt.local_point;
        cache.normal_lambda = pt.normal_lambda;
        cache.friction1_lambda = pt.friction1_lambda;
        cache.friction2_lambda = pt.friction2_lambda;
    }

    m_point_count = 0;
}

void ContactManifold::end_update() {
    for (auto& pt: *this) {
        float min_sqr_dist = CACHE_THRESHOLD * CACHE_THRESHOLD;
        int cache_idx = -1;

        for (int i = 0; i < m_lambda_cache_size; i++) {
            float dist = pt.local_point.square_distance(m_lambda_cache[i].local_point);
            if (dist < min_sqr_dist) {
                min_sqr_dist = dist;
                cache_idx = i;
            }
        }

        if (cache_idx != -1) {
            auto& cache = m_lambda_cache[cache_idx];
            pt.normal_lambda = cache.normal_lambda;
            pt.friction1_lambda = cache.friction1_lambda;
            pt.friction2_lambda = cache.friction2_lambda;

            m_lambda_cache[cache_idx] = m_lambda_cache[--m_lambda_cache_size];

        } else {
            pt.normal_lambda = 0.f;
            pt.friction1_lambda = 0.f;
            pt.friction2_lambda = 0.f;
        }

        pt.normal_constr_id = -1;
        pt.friction1_constr_id = -1;
        pt.friction2_constr_id = -1;
    }
}

void ContactManifold::add_contact(const ContactGeom& new_geom) {
    static_assert(MAX_SIZE == 4);

    auto local_point = m_shape1_inv_transform.apply_point(new_geom.p1);
    float penetration = new_geom.normal.dot(new_geom.p1 - new_geom.p2);

    if (m_point_count < MAX_SIZE) {
        reset_point(m_points[m_point_count++], new_geom, local_point, penetration);
        return;
    }

    float max_pen = penetration;
    int max_pen_idx = -1;
    for (int i = 0; i < MAX_SIZE; i++) {
        if (m_points[i].penetration > max_pen) {
            max_pen = m_points[i].penetration;
            max_pen_idx = i;
        }
    }

    vec3 pts[] = {
            m_points[0].local_point,
            m_points[1].local_point,
            m_points[2].local_point,
            m_points[3].local_point};

    float max_area = -FLOAT_MAX;

    if (max_pen_idx != -1)
        max_area = evaluate_area(pts[0], pts[1], pts[2], pts[3]);

    int max_area_idx = -1;

    for (int i = 0; i < MAX_SIZE; i++) {
        if (max_pen_idx != i) {
            pts[i] = local_point;
            float area = evaluate_area(pts[0], pts[1], pts[2], pts[3]);
            pts[i] = m_points[i].local_point;
            if (area > max_area) {
                max_area = area;
                max_area_idx = i;
            }
        }
    }

    if (max_area_idx != -1) {
        reset_point(m_points[max_area_idx], new_geom, local_point, penetration);
    }
}

} // slope
