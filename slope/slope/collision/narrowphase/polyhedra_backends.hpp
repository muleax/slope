#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"

namespace slope {

template <class A, class B>
class PolyhedraBackendBase : public NpBackend<A, B> {
protected:
    void generate_contact_patch(const A* shape1, const B* shape2, const Vec3& pen_axis, NpContactPatch& patch)
    {
        auto* ctx = this->ctx();

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
            patch.invert_input_order();
        }

        ctx->face_clipper.clip_convex_face_by_convex_prism(
            ctx->clipped_face, ctx->support_face[face],
            ctx->support_face[clipper],
            clipper_axis);

        for (auto& clipped_pt: ctx->clipped_face) {
            auto t = Plane(support_normal[clipper], ctx->support_face[clipper][0]).intersect_ray(clipped_pt, clipper_axis);
            if (t) {
                auto p1 = clipped_pt + clipper_axis * *t;
                patch.contacts.push_back({p1, clipped_pt, clipper_axis});
            }
        }
    }
};

template <class A, class B>
class GJKPolyhedraBackend : public PolyhedraBackendBase<A, B> {
public:
    bool intersect(const A* shape1, const B* shape2)
    {
        return this->ctx()->gjk_solver.intersect(shape1, shape2);
    }

    bool collide(const A* shape1, const B* shape2, NpContactPatch& patch)
    {
        if (this->intersect(shape1, shape2)) {
            auto pen_axis = this->ctx()->epa_solver.find_penetration_axis(shape1, shape2, this->ctx()->gjk_solver.simplex());
            if (pen_axis)
                this->generate_contact_patch(shape1, shape2, *pen_axis, patch);

            return true;
        }

        return false;
    }
};

template <class A, class B>
class SATPolyhedraBackend : public PolyhedraBackendBase<A, B> {
public:
    bool intersect(const A* shape1, const B* shape2)
    {
        return this->ctx()->sat_solver.find_penetration_axis(shape1, shape2).has_value();
    }

    bool collide(const A* shape1, const B* shape2, NpContactPatch& patch)
    {
        auto pen_axis = this->ctx()->sat_solver.find_penetration_axis(shape1, shape2);
        if (pen_axis) {
            this->generate_contact_patch(shape1, shape2, *pen_axis, patch);
            return true;
        }

        return false;
    }
};

using GJKConvexPolyhedronBackend = GJKPolyhedraBackend<PolyhedronShape, PolyhedronShape>;
using GJKConvexPolyhedronBoxBackend = GJKPolyhedraBackend<PolyhedronShape, BoxShape>;
using GJKBoxBackend = GJKPolyhedraBackend<BoxShape, BoxShape>;

using SATConvexPolyhedronBackend = SATPolyhedraBackend<PolyhedronShape, PolyhedronShape>;
using SATConvexPolyhedronBoxBackend = SATPolyhedraBackend<PolyhedronShape, BoxShape>;
using SATBoxBackend = SATPolyhedraBackend<BoxShape, BoxShape>;

} // slope
