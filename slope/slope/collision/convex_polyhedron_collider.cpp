#include "convex_polyhedron_collider.hpp"

namespace slope {

namespace {

struct IntervalPenetration {
    float depth = FLOAT_MAX;
    float direction = 1.f;
};

inline IntervalPenetration get_interval_penetration(const Interval& a, const Interval& b) {
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

void ConvexPolyhedronCollider::clip_convex_face_by_halfspace(
        Vector<Vec3>& out_clipped_face,
        const Vector<Vec3>& face, const Plane& hs_plane) {

    const float DIVISION_EPSILON = 1e-8f;
    const float BLOAT_EPSILON = 1e-4f;

    out_clipped_face.clear();

    const Vec3* prev_pt = &face.back();
    float prev_dot = hs_plane.normal.dot(face.back());
    float prev_delta_dot = prev_dot - hs_plane.dot;

    for (auto& pt : face) {
        float dot = hs_plane.normal.dot(pt);
        float delta_dot = dot - hs_plane.dot;

        if (delta_dot * prev_delta_dot < 0.f) {
            float divider = prev_dot - dot;
            if(divider * divider > DIVISION_EPSILON) {
                float t = (hs_plane.dot - dot) / divider;
                out_clipped_face.push_back(pt + t * (*prev_pt - pt));
            }
        }

        if (delta_dot > -BLOAT_EPSILON)
            out_clipped_face.push_back(pt);

        prev_dot = dot;
        prev_delta_dot = delta_dot;
        prev_pt = &pt;
    }
}

void ConvexPolyhedronCollider::clip_convex_face_by_convex_prism(
        Vector<Vec3>& out_clipped_face,
        const Vector<Vec3>& face, const Vector<Vec3>& prism_base, const Vec3& prism_axis) {

    // TODO: optimize
    m_clip_buffer = face;

    Vec3 base_cross = (prism_base[0] - prism_base.back()).cross(prism_base[1] - prism_base[0]);
    float denom = prism_axis.dot(base_cross);
    Vec3 prism_dir = denom > 0.f ? -prism_axis : prism_axis;

    const Vec3* prev_base_pt = &prism_base.back();

    for (auto& base_pt : prism_base) {
        auto base_edge = base_pt - *prev_base_pt;
        auto hs_normal = base_edge.cross(prism_dir).normalized();

        clip_convex_face_by_halfspace(out_clipped_face, m_clip_buffer, {hs_normal, base_pt});
        if(out_clipped_face.empty())
            break;

        prev_base_pt = &base_pt;
        std::swap(out_clipped_face, m_clip_buffer);
    }
}

bool ConvexPolyhedronCollider::get_penetration_axis(
        Vec3& out_pen_axis,
        const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2) {

    const float EDGE_CROSS_EPSILON = 1e-12f;

    IntervalPenetration min_pen;

    auto sat_test = [&out_pen_axis, &min_pen, shape1, shape2](const Vec3& axis) -> bool {
        Interval itv1 = shape1->project(axis);
        Interval itv2 = shape2->project(axis);
        if(!itv1.intersects(itv2))
            return false;

        auto pen = get_interval_penetration(itv1, itv2);
        if(pen.depth < min_pen.depth) {
            out_pen_axis = axis;
            min_pen = pen;
        }

        return true;
    };

    for (auto& normal : shape1->unique_world_face_normals()) {
        if (!sat_test(normal))
            return false;
    }

    for (auto& normal : shape2->unique_world_face_normals()) {
        if (!sat_test(normal))
            return false;
    }

    for (auto& edge1 : shape1->unique_world_edge_dirs()) {
        for(auto& edge2 : shape2->unique_world_edge_dirs()) {
            auto axis = edge1.cross(edge2);
            float axis_sqr_len = axis.length_squared();
            if(axis_sqr_len < EDGE_CROSS_EPSILON)
                continue;

            axis /= sqrtf(axis_sqr_len);

            if (!sat_test(axis))
                return false;
        }
    }

    out_pen_axis *= min_pen.direction;
    return true;
}

bool ConvexPolyhedronCollider::collide(
        Vector<ContactGeom>& out_contacts,
        const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2 ) {

    Vec3 pen_axis;
    if (!get_penetration_axis(pen_axis, shape1, shape2))
        return false;

    float proximity[2];
    Vec3 support_normal[2];

    m_support_face[0].clear();
    m_support_face[1].clear();
    m_clipped_face.clear();
    out_contacts.clear();

    proximity[0] = shape1->get_support_face(pen_axis, m_support_face[0], support_normal[0]);
    proximity[1] = shape2->get_support_face(-pen_axis, m_support_face[1], support_normal[1]);

    if(proximity[0] > proximity[1]) {
        clip_convex_face_by_convex_prism(m_clipped_face, m_support_face[1], m_support_face[0], pen_axis);

        for (auto& clipped_pt : m_clipped_face) {
            float t;
            if (Plane(support_normal[0], m_support_face[0][0]).intersect_ray(t, clipped_pt, pen_axis)) {
                auto p1 = clipped_pt + pen_axis * t;
                out_contacts.push_back({p1, clipped_pt, pen_axis});
            }
        }
    } else {
        clip_convex_face_by_convex_prism(m_clipped_face, m_support_face[0], m_support_face[1], pen_axis);

        for (auto& clipped_pt : m_clipped_face) {
            float t;
            if (Plane(support_normal[1], m_support_face[1][0]).intersect_ray(t, clipped_pt, -pen_axis)) {
                auto p2 = clipped_pt - pen_axis * t;
                out_contacts.push_back({clipped_pt, p2, pen_axis});
            }
        }
    }

    return !out_contacts.empty();
}

} // slope
