#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"

namespace slope {

template <class A, class B>
class PolyhedraBackendBase : public NpBackend<A, B> {
protected:
    void generate_contacts_impl(ContactManifold& manifold, const Vec3& pen_axis)
    {
        auto* ctx = this->context();

        float proximity[2];
        Vec3 support_normal[2];

        ctx->support_face[0].clear();
        ctx->support_face[1].clear();
        ctx->clipped_face.clear();

        proximity[0] = this->shape1()->get_support_face(pen_axis, ctx->support_face[0], support_normal[0]);
        proximity[1] = this->shape2()->get_support_face(-pen_axis, ctx->support_face[1], support_normal[1]);

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

        ctx->face_clipper.clip_convex_face_by_convex_prism(
            ctx->clipped_face, ctx->support_face[face],
            ctx->support_face[clipper],
            clipper_axis);

        for (auto& clipped_pt: ctx->clipped_face) {
            auto t = Plane(support_normal[clipper], ctx->support_face[clipper][0]).intersect_ray(clipped_pt, clipper_axis);
            if (t) {
                auto p1 = clipped_pt + clipper_axis * *t;
                manifold.add_contact({p1, clipped_pt, clipper_axis});
            }
        }
    }
};

template <class A, class B>
class GJKPolyhedraBackend : public PolyhedraBackendBase<A, B> {
public:
    bool intersect()
    {
        return this->context()->gjk_solver.intersect(this->shape1(), this->shape2());
    }

    std::optional<Vec3> get_penetration_axis()
    {
        auto* ctx = this->context();
        return ctx->epa_solver.find_penetration_axis(this->shape1(), this->shape2(), ctx->gjk_solver.simplex());
    }

    void generate_contacts(ContactManifold& manifold)
    {
        auto pen_axis = this->get_penetration_axis();
        if (pen_axis)
            this->generate_contacts_impl(manifold, *pen_axis);
    }
};

template <class A, class B>
class SATPolyhedraBackend : public PolyhedraBackendBase<A, B> {
public:
    bool intersect()
    {
        m_min_pen_axis = this->context()->sat_solver.find_penetration_axis(this->shape1(), this->shape2());
        return m_min_pen_axis.has_value();
    }

    std::optional<Vec3> get_penetration_axis()
    {
        return m_min_pen_axis;
    }

    void generate_contacts(ContactManifold& manifold)
    {
        this->generate_contacts_impl(manifold, *m_min_pen_axis);
    }

private:
    std::optional<Vec3> m_min_pen_axis;
};

using GJKConvexPolyhedronBackend = GJKPolyhedraBackend<PolyhedronShape, PolyhedronShape>;
using GJKConvexPolyhedronBoxBackend = GJKPolyhedraBackend<PolyhedronShape, BoxShape>;
using GJKBoxBackend = GJKPolyhedraBackend<BoxShape, BoxShape>;

using SATConvexPolyhedronBackend = SATPolyhedraBackend<PolyhedronShape, PolyhedronShape>;
using SATConvexPolyhedronBoxBackend = SATPolyhedraBackend<PolyhedronShape, BoxShape>;
using SATBoxBackend = SATPolyhedraBackend<BoxShape, BoxShape>;

} // slope
