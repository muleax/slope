#include "slope/collision/face_clipper.hpp"

namespace slope {


void FaceClipper::clip_convex_face_by_halfspace(
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
                float t = -delta_dot / divider;
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

void FaceClipper::clip_convex_face_by_convex_prism(
    Vector<Vec3>& out_clipped_face,
    const Vector<Vec3>& face, const Vector<Vec3>& prism_base, const Vec3& prism_axis) {

    // TODO: optimize
    m_buffer = face;

    Vec3 base_cross = (prism_base[0] - prism_base.back()).cross(prism_base[1] - prism_base[0]);
    float denom = prism_axis.dot(base_cross);
    Vec3 prism_dir = denom > 0.f ? -prism_axis : prism_axis;

    const Vec3* prev_base_pt = &prism_base.back();

    for (auto& base_pt : prism_base) {
        auto base_edge = base_pt - *prev_base_pt;
        auto hs_normal = base_edge.cross(prism_dir).normalized();

        clip_convex_face_by_halfspace(out_clipped_face, m_buffer, {hs_normal, base_pt});
        if(out_clipped_face.empty())
            break;

        prev_base_pt = &base_pt;
        std::swap(out_clipped_face, m_buffer);
    }

    std::swap(out_clipped_face, m_buffer);
}

LineSegment FaceClipper::clip_segment_by_convex_prism(
    const LineSegment& segment, const Vector<Vec3>& prism_base, const Vec3& prism_axis)
{
    const float DIVISION_EPSILON = 1e-8f;

    LineSegment clipped_segment = segment;

    Vec3 base_cross = (prism_base[0] - prism_base.back()).cross(prism_base[1] - prism_base[0]);
    float denom = prism_axis.dot(base_cross);
    Vec3 prism_dir = denom > 0.f ? -prism_axis : prism_axis;

    const Vec3* prev_base_pt = &prism_base.back();

    for (auto& base_pt : prism_base) {
        auto base_edge = base_pt - *prev_base_pt;
        auto hs_normal = base_edge.cross(prism_dir).normalized();
        Plane hs_plane(hs_normal, base_pt);

        float prev_dot = hs_plane.normal.dot(clipped_segment.begin);
        float prev_delta_dot = prev_dot - hs_plane.dot;

        float dot = hs_plane.normal.dot(clipped_segment.end);
        float delta_dot = dot - hs_plane.dot;

        if (delta_dot * prev_delta_dot < 0.f) {
            float divider = prev_dot - dot;
            if(divider * divider > DIVISION_EPSILON) {
                float t = -delta_dot / divider;
                Vec3 mp = clipped_segment.end + t * (clipped_segment.begin - clipped_segment.end);
                if (delta_dot > 0.f)
                    clipped_segment = {mp, clipped_segment.end};
                else
                    clipped_segment = {clipped_segment.begin, mp};
            }
        }

        prev_base_pt = &base_pt;
    }

    return clipped_segment;
}

} // slope
