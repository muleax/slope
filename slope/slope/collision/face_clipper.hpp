#pragma once
#include "slope/math/vector3.hpp"
#include "slope/containers/vector.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

class FaceClipper {
public:
    void clip_convex_face_by_halfspace(
        Vector<Vec3>& out_clipped_face,
        const Vector<Vec3>& face, const Plane& hs_plane);

    void clip_convex_face_by_convex_prism(
        Vector<Vec3>& out_clipped_face,
        const Vector<Vec3>& face, const Vector<Vec3>& prism_base, const Vec3& prism_axis);

private:
    Vector<Vec3> m_buffer;
};

} // slope
