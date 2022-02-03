#pragma once
#include "slope/math/vector3.hpp"
#include "slope/containers/vector.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

class FaceClipper {
public:
    void clip_convex_face_by_halfspace(
        Vector<vec3>& out_clipped_face,
        const Vector<vec3>& face, const Plane& hs_plane);

    void clip_convex_face_by_convex_prism(
        Vector<vec3>& out_clipped_face,
        const Vector<vec3>& face, const Vector<vec3>& prism_base, const vec3& prism_axis);

    LineSegment clip_segment_by_convex_prism(
        const LineSegment& segment, const Vector<vec3>& prism_base, const vec3& prism_axis);

private:
    Vector<vec3> m_buffer;
};

} // slope
