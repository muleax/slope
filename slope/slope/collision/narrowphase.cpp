#include "slope/collision/narrowphase.hpp"

namespace slope {

void Narrowphase::generate_contacts(
    ContactManifold& manifold, const Vec3& pen_axis,
    const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2)
{
    float proximity[2];
    Vec3 support_normal[2];

    m_support_face[0].clear();
    m_support_face[1].clear();
    m_clipped_face.clear();

    proximity[0] = shape1->get_support_face(pen_axis, m_support_face[0], support_normal[0]);
    proximity[1] = shape2->get_support_face(-pen_axis, m_support_face[1], support_normal[1]);

    if(proximity[0] > proximity[1]) {
        m_face_clipper.clip_convex_face_by_convex_prism(m_clipped_face, m_support_face[1], m_support_face[0], pen_axis);

        for (auto& clipped_pt : m_clipped_face) {
            float t;
            if (Plane(support_normal[0], m_support_face[0][0]).intersect_ray(t, clipped_pt, pen_axis)) {
                auto p1 = clipped_pt + pen_axis * t;
                manifold.add_contact({p1, clipped_pt, pen_axis});
            }
        }
    } else {
        m_face_clipper.clip_convex_face_by_convex_prism(m_clipped_face, m_support_face[0], m_support_face[1], pen_axis);

        for (auto& clipped_pt : m_clipped_face) {
            float t;
            if (Plane(support_normal[1], m_support_face[1][0]).intersect_ray(t, clipped_pt, -pen_axis)) {
                auto p2 = clipped_pt - pen_axis * t;
                manifold.add_contact({clipped_pt, p2, pen_axis});
            }
        }
    }
}

std::optional<Vec3> Narrowphase::find_penetration_axis(const CollisionShape* shape1, const CollisionShape* shape2) {
    if (m_preferred_backend == Backend::SAT) {
        if (shape1->type() == ShapeType::ConvexPolyhedron && shape2->type() == ShapeType::ConvexPolyhedron) {
            return m_sat_solver.find_penetration_axis(
                static_cast<const ConvexPolyhedronShape*>(shape1), static_cast<const ConvexPolyhedronShape*>(shape2));
        }
    }

    if (m_gjk_solver.intersect(shape1, shape2))
        return m_epa_solver.find_penetration_axis(shape1, shape2, m_gjk_solver.simplex());

    return std::nullopt;
}

void Narrowphase::generate_contacts(
    ContactManifold& manifold, const Vec3& pen_axis,
    const CollisionShape* shape1, const CollisionShape* shape2)
{
    switch (shape1->type()) {

    case ShapeType::ConvexPolyhedron:

        switch (shape2->type()) {

        case ShapeType::ConvexPolyhedron:
            generate_contacts(
                manifold, pen_axis,
                static_cast<const ConvexPolyhedronShape*>(shape1), static_cast<const ConvexPolyhedronShape*>(shape2));
            break;

        case ShapeType::Sphere:
            break;

        default:
            SL_VERIFY(false);
        }
        break;

    case ShapeType::Sphere:

        switch (shape2->type()) {

        case ShapeType::ConvexPolyhedron:
            break;

        case ShapeType::Sphere:
            break;

        default:
            SL_VERIFY(false);
        }
        break;

    default:
        SL_VERIFY(false);
    }
}

} // slope
