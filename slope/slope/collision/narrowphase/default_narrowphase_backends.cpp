#include "slope/collision/narrowphase/default_narrowphase_backends.hpp"

namespace slope {

template <class Shape1, class Shape2>
static void generate_polyhedra_contacts(
    NpContext* ctx, ContactManifold& manifold, const Shape1* shape1, const Shape2* shape2, const Vec3& pen_axis)
{
    float proximity[2];
    Vec3 support_normal[2];

    ctx->support_face[0].clear();
    ctx->support_face[1].clear();
    ctx->clipped_face.clear();

    proximity[0] = shape1->get_support_face(pen_axis, ctx->support_face[0], support_normal[0]);
    proximity[1] = shape2->get_support_face(-pen_axis, ctx->support_face[1], support_normal[1]);

    int face;
    int clipper;
    Vec3 clipper_axis;

    if (proximity[0] < proximity[1]) {
        face = 1;
        clipper = 0;
        clipper_axis = pen_axis;
    } else {
        face = 0;
        clipper = 1;
        clipper_axis = -pen_axis;
        manifold.invert_input_order();
    }

    ctx->face_clipper.clip_convex_face_by_convex_prism(ctx->clipped_face, ctx->support_face[face],
                                                       ctx->support_face[clipper],
                                                       clipper_axis);

    for (auto& clipped_pt: ctx->clipped_face) {
        float t;
        if (Plane(support_normal[clipper], ctx->support_face[clipper][0]).intersect_ray(t, clipped_pt, clipper_axis)) {
            auto p1 = clipped_pt + clipper_axis * t;
            manifold.add_contact({p1, clipped_pt, clipper_axis});
        }
    }
}

void SATConvexPolyhedronBackend::generate_contacts(ContactManifold& manifold)
{
    generate_polyhedra_contacts(context(), manifold, shape1(), shape2(), *m_min_pen_axis);
}

void GJKConvexPolyhedronBackend::generate_contacts(ContactManifold& manifold)
{
    auto pen_axis = get_penetration_axis();
    if (pen_axis)
        generate_polyhedra_contacts(context(), manifold, shape1(), shape2(), *pen_axis);
}

void ConvexPolyhedronCapsuleBackend::generate_contacts(ContactManifold& manifold)
{
    auto pen_axis = get_penetration_axis();
    if (!pen_axis)
        return;

    // TODO: robust solution for imprecise penetration axis
    auto* ctx = context();
    auto& support_face = ctx->support_face[0];
    support_face.clear();

    Vec3 support_normal;
    shape1()->get_support_face(*pen_axis, support_face, support_normal);

    auto segment = ctx->face_clipper.clip_segment_by_convex_prism(
        shape2()->segment(), support_face, *pen_axis);

    for (int i = 0; i < 2; i++) {
        auto clipped_pt = segment[i] - *pen_axis * shape2()->radius();
        float t;
        if (Plane(support_normal, support_face[0]).intersect_ray(t, clipped_pt, *pen_axis)) {
            auto p1 = clipped_pt + t * *pen_axis;
            manifold.add_contact({p1, clipped_pt, *pen_axis});
        }
    }
}

void ConvexPolyhedronSphereBackend::generate_contacts(ContactManifold& manifold)
{
    auto pen_axis = get_penetration_axis();
    if (!pen_axis)
        return;

    // TODO: robust solution for imprecise penetration axis
    // TODO: optimize
    auto* ctx = context();
    Vec3 face_normal;
    ctx->support_face[0].clear();
    shape1()->get_support_face(*pen_axis, ctx->support_face[0], face_normal);

    auto p2 = shape2()->support_point(-*pen_axis, 0.f);
    float t;
    Plane(face_normal, ctx->support_face[0][0]).intersect_ray(t, p2, *pen_axis);
    auto p1 = p2 + t * *pen_axis;
    manifold.add_contact({p1, p2, *pen_axis});
}

} // slope
