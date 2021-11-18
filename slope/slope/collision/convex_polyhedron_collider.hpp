#pragma once
#include "slope/collision/convex_polyhedron_shape.hpp"
#include "slope/collision/primitives.hpp"
#include "slope/containers/vector.hpp"

namespace slope {

class ConvexPolyhedronCollider {
public:
    bool get_penetration_axis(
            Vec3& out_pen_axis,
            const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);

    bool collide(
            Vector<ContactGeom>& out_contacts,
            const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);

private:
    Vector<Vec3> m_support_face[2];
    Vector<Vec3> m_clipped_face;
    Vector<Vec3> m_clip_buffer;

    void clip_convex_face_by_halfspace(
            Vector<Vec3>& out_clipped_face,
            const Vector<Vec3>& face, const Plane& hs_plane);

    void clip_convex_face_by_convex_prism(
            Vector<Vec3>& out_clipped_face,
            const Vector<Vec3>& face, const Vector<Vec3>& prism_base, const Vec3& prism_axis);
};

} // slope