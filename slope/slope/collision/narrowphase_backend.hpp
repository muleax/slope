#pragma once
#include "slope/collision/collision_shape.hpp"
#include "slope/collision/gjk.hpp"
#include "slope/collision/sat.hpp"
#include "slope/collision/epa.hpp"
#include "slope/collision/face_clipper.hpp"
#include "slope/collision/contact_manifold.hpp"

#include "slope/collision/convex_polyhedron_shape.hpp"
#include "slope/collision/sphere_shape.hpp"

namespace slope {

struct NpContext {
    GJKSolver gjk_solver;
    EPASolver epa_solver;
    SATSolver sat_solver;

    FaceClipper face_clipper;
    Vector<Vec3> support_face[2];
    Vector<Vec3> clipped_face;

    std::optional<Vec3> min_pen_axis;
};

enum class NpBackendHint : int {
    GJK_EPA = 0,
    SAT,
    Count
};

// Derive specialization in order to support collision pair
template <class A, class B, NpBackendHint Hint = NpBackendHint::GJK_EPA>
struct NpBackend {
    using Shape1 = A;
    using Shape2 = B;
    static constexpr NpBackendHint Hint_ = Hint;

    // Expected interface:
    // static bool intersect(NpContext& ctx, const A* shape1, const B* shape2);
    // static void generate_contacts(NpContext& ctx, ContactManifold& manifold, const A* shape1, const B* shape2);
};

struct NullBackend {
    static bool intersect(NpContext& ctx, const CollisionShape* shape1, const CollisionShape* shape2) { return false; }
    static void generate_contacts(NpContext& ctx, ContactManifold& manifold, const CollisionShape* shape1, const CollisionShape* shape2) {}
};

template <class Backend, bool Inverted = false>
struct NpBackendWrapper {
    using Shape1 = typename Backend::Shape1;
    using Shape2 = typename Backend::Shape2;

    static bool intersect(NpContext& ctx, const CollisionShape* shape1, const CollisionShape* shape2)
    {
        if constexpr (Inverted)
            return Backend::intersect(ctx, (const Shape1*)shape2, (const Shape2*)shape1);
        else
            return Backend::intersect(ctx, (const Shape1*)shape1, (const Shape2*)shape2);
    }

    static void generate_contacts(
        NpContext& ctx, ContactManifold& manifold,
        const CollisionShape* shape1, const CollisionShape* shape2)
    {
        if constexpr (Inverted) {
            manifold.invert_input_order();
            Backend::generate_contacts(ctx, manifold, (const Shape1*)shape2, (const Shape2*)shape1);
        } else {
            Backend::generate_contacts(ctx, manifold, (const Shape1*)shape1, (const Shape2*)shape2);
        }
    }
};

template <NpBackendHint Hint>
struct ConvexPolyhedronBackend : NpBackend<ConvexPolyhedronShape, ConvexPolyhedronShape, Hint> {

    static bool intersect(NpContext& ctx, const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2)
    {
        if constexpr (Hint == NpBackendHint::GJK_EPA) {
            return ctx.gjk_solver.intersect(shape1, shape2);
        } else {
            ctx.min_pen_axis = ctx.sat_solver.find_penetration_axis(shape1, shape2);
            return ctx.min_pen_axis.has_value();
        }
    }

    static void generate_contacts(
        NpContext& ctx, ContactManifold& manifold,
        const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2)
    {
        Vec3 pen_axis;
        if constexpr (Hint == NpBackendHint::GJK_EPA) {
            pen_axis = ctx.epa_solver.find_penetration_axis(shape1, shape2, ctx.gjk_solver.simplex());
        } else {
            pen_axis = *ctx.min_pen_axis;
        }

        float proximity[2];
        Vec3 support_normal[2];

        ctx.support_face[0].clear();
        ctx.support_face[1].clear();
        ctx.clipped_face.clear();

        proximity[0] = shape1->get_support_face(pen_axis, ctx.support_face[0], support_normal[0]);
        proximity[1] = shape2->get_support_face(-pen_axis, ctx.support_face[1], support_normal[1]);

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

        ctx.face_clipper.clip_convex_face_by_convex_prism(ctx.clipped_face, ctx.support_face[face], ctx.support_face[clipper],
                                                        clipper_axis);

        for (auto& clipped_pt: ctx.clipped_face) {
            float t;
            if (Plane(support_normal[clipper], ctx.support_face[clipper][0]).intersect_ray(t, clipped_pt, clipper_axis)) {
                auto p1 = clipped_pt + clipper_axis * t;
                manifold.add_contact({p1, clipped_pt, clipper_axis});
            }
        }
    }
};

struct ConvexPolyhedronSphereBackend : NpBackend<ConvexPolyhedronShape, SphereShape> {

    static bool intersect(NpContext& ctx, const ConvexPolyhedronShape* shape1, const SphereShape* shape2)
    {
        return ctx.gjk_solver.intersect(shape1, shape2);
    }

    static void generate_contacts(
        NpContext& ctx, ContactManifold& manifold,
        const ConvexPolyhedronShape* shape1, const SphereShape* shape2)
    {
        auto pen_axis = ctx.epa_solver.find_penetration_axis(shape1, shape2, ctx.gjk_solver.simplex());

        // TODO: optimize
        Vec3 face_normal;
        ctx.support_face[0].clear();
        shape1->get_support_face(pen_axis, ctx.support_face[0], face_normal);

        auto p2 = shape2->support_point(-pen_axis);
        float t;
        Plane(face_normal, ctx.support_face[0][0]).intersect_ray(t, p2, pen_axis);
        auto p1 = p2 + t *  pen_axis;
        manifold.add_contact({p1, p2, pen_axis});
    }
};

struct SphereBackend : NpBackend<SphereShape, SphereShape> {

    static bool intersect(NpContext& ctx, const SphereShape* shape1, const SphereShape* shape2)
    {
        const auto& p1 = shape1->transform().translation();
        const auto& p2 = shape2->transform().translation();
        float dist = p1.distance(p2);
        if (dist < shape1->radius() + shape2->radius()) {
            if (dist > 1e-6f)
                ctx.min_pen_axis = (p2 - p1) / (dist);
            else
                ctx.min_pen_axis = Vec3{1.f, 0.f, 0.f};

            return true;
        }

        return false;
    }

    static void generate_contacts(
        NpContext& ctx, ContactManifold& manifold,
        const SphereShape* shape1, const SphereShape* shape2)
    {
        const auto& p1 = shape1->transform().translation();
        const auto& p2 = shape2->transform().translation();
        auto& pen_axis = *ctx.min_pen_axis;

        manifold.add_contact({p1 + pen_axis * shape1->radius(), p2 - pen_axis * shape2->radius(), pen_axis});
    }
};
/*
template <class A, class B, NpBackendHint Hint>
bool NpBackend<A, B, Hint>::intersect(NpContext& ctx, const A* shape1, const B* shape2)
{
    using Inversion = NpBackend<B, A, Hint>;
    using GJKFallback = NpBackend<A, B, NpBackendHint::GJK_EPA>;
    using GJKInversionFallback = NpBackend<B, A, NpBackendHint::GJK_EPA>;

    if constexpr (!std::is_base_of_v<NotImplemented, Inversion>)
        return Inversion::intersect(ctx, shape2, shape1);

    else if constexpr (!std::is_base_of_v<NotImplemented, GJKFallback>)
        return GJKFallback::intersect(ctx, shape1, shape2);

    else if constexpr (!std::is_base_of_v<NotImplemented, GJKInversionFallback>)
        return GJKInversionFallback::intersect(ctx, shape2, shape1);

    else
        return false;
}

template <class A, class B, NpBackendHint Hint>
void NpBackend<A, B, Hint>::generate_contacts(NpContext& ctx, ContactManifold& manifold, const A* shape1, const B* shape2)
{
    using Inversion = NpBackend<B, A, Hint>;
    using GJKFallback = NpBackend<A, B, NpBackendHint::GJK_EPA>;
    using GJKInversionFallback = NpBackend<B, A, NpBackendHint::GJK_EPA>;

    if constexpr (!std::is_base_of_v<NotImplemented, Inversion>) {
        manifold.invert_input_order();
        Inversion::generate_contacts(ctx, manifold, shape2, shape1);
    }

    else if constexpr (!std::is_base_of_v<NotImplemented, GJKFallback>) {
        return GJKFallback::generate_contacts(ctx, manifold, shape1, shape2);
    }

    else if constexpr (!std::is_base_of_v<NotImplemented, GJKInversionFallback>) {
        manifold.invert_input_order();
        GJKInversionFallback::generate_contacts(ctx, manifold, shape2, shape1);
    }
}

*/

} // slope
