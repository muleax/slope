#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"

namespace slope {

template <class A, class B>
class PolyhedraSphereBackend : public NpBackend<A, B> {
public:
    bool intersect(const A* shape1, const B* shape2)
    {
        return this->ctx()->gjk_solver.intersect(shape1, shape2);
    }

    bool collide(const A* shape1, const B* shape2, NpContactPatch& patch)
    {
        if (!this->intersect(shape1, shape2))
            return false;

        auto* ctx = this->ctx();
        auto pen_axis = ctx->epa_solver.find_penetration_axis(shape1, shape2, ctx->gjk_solver.simplex());
        if (pen_axis) {
            // TODO: robust solution for imprecise penetration axis
            // TODO: optimize
            vec3 face_normal;
            ctx->support_face[0].clear();
            shape1->get_support_face(*pen_axis, ctx->support_face[0], face_normal);

            auto p2 = shape2->support(-*pen_axis, 0.f, true);
            auto t = Plane(face_normal, ctx->support_face[0][0]).intersect_ray(p2, *pen_axis);
            if (t) {
                auto p1 = p2 + *t * *pen_axis;
                patch.contacts.push_back({p1, p2, *pen_axis});
            }
        }

        return true;
    }
};

using ConvexPolyhedronSphereBackend = PolyhedraSphereBackend<PolyhedronShape, SphereShape>;
using BoxSphereBackend = PolyhedraSphereBackend<BoxShape, SphereShape>;

class SphereBackend : public NpBackend<SphereShape, SphereShape> {
public:
    bool intersect(const Shape1* shape1, const Shape2* shape2)
    {
        float sqr_dist;
        return intersect_impl(shape1, shape2, sqr_dist);
    }

    bool collide(const Shape1* shape1, const Shape2* shape2, NpContactPatch& patch)
    {
        static constexpr float DIST_EPSILON = 1e-6f;

        float sqr_dist;
        if (!intersect_impl(shape1, shape2, sqr_dist))
            return false;

        float dist = sqrtf(sqr_dist);
        auto& center1 = shape1->transform().translation();
        auto& center2 = shape2->transform().translation();
        auto pen_axis = dist > DIST_EPSILON ? (center2 - center1) / dist : vec3{1.f, 0.f, 0.f};
        auto p1 = center1 + pen_axis * shape1->radius();
        auto p2 = center2 - pen_axis * shape2->radius();
        patch.contacts.push_back({ p1, p2, pen_axis });

        return true;
    }

private:
    bool intersect_impl(const Shape1* shape1, const Shape2* shape2, float& sqr_dist)
    {
        auto& p1 = shape1->transform().translation();
        auto& p2 = shape2->transform().translation();
        float contact_dist = shape1->radius() + shape2->radius();
        sqr_dist = p1.square_distance(p2);
        return sqr_dist <= contact_dist * contact_dist;
    }

    vec3 m_p1;
    vec3 m_p2;
    float m_dist_sqr = 0.f;
};

} // slope
