#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/capsule_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"

namespace slope {

template <class A, class B>
class PolyhedraCapsuleBackend : public NpBackend<A, B> {
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
            auto& support_face = ctx->support_face[0];
            support_face.clear();

            vec3 support_normal;
            shape1->get_support_face(*pen_axis, support_face, support_normal);

            auto segment = ctx->face_clipper.clip_segment_by_convex_prism(shape2->segment(), support_face, *pen_axis);

            for (int i = 0; i < 2; i++) {
                auto clipped_pt = segment[i] - *pen_axis * shape2->radius();
                auto t = Plane(support_normal, support_face[0]).intersect_ray(clipped_pt, *pen_axis);
                if (t) {
                    auto p1 = clipped_pt + *t * *pen_axis;
                    patch.contacts.push_back({p1, clipped_pt, *pen_axis});
                }
            }
        }

        return true;
    }
};

using ConvexPolyhedronCapsuleBackend = PolyhedraCapsuleBackend<PolyhedronShape, CapsuleShape>;
using BoxCapsuleBackend = PolyhedraCapsuleBackend<BoxShape, CapsuleShape>;

class CapsuleSphereBackend : public NpBackend<CapsuleShape, SphereShape> {
public:
    bool intersect(const Shape1* shape1, const Shape2* shape2)
    {
        vec3 closest_pt;
        float sqr_dist;
        return intersect_impl(shape1, shape2, closest_pt, sqr_dist);
    }

    bool collide(const Shape1* shape1, const Shape2* shape2, NpContactPatch& patch)
    {
        static constexpr float DIST_EPSILON = 1e-6f;

        vec3 closest_pt;
        float sqr_dist;
        if (!intersect_impl(shape1, shape2, closest_pt, sqr_dist))
            return false;

        float dist = sqrtf(sqr_dist);
        auto& sphere_center = shape2->transform().translation();
        auto pen_axis = (dist > DIST_EPSILON) ? (sphere_center - closest_pt) / dist : vec3{1.f, 0.f, 0.f};
        auto p1 = closest_pt + pen_axis * shape1->radius();
        auto p2 = sphere_center - pen_axis * shape2->radius();
        patch.contacts.push_back({p1, p2, pen_axis});

        return true;
    }

private:
    bool intersect_impl(const Shape1* shape1, const Shape2* shape2, vec3& closest_pt, float& sqr_dist)
    {
        auto& sphere_center = shape2->transform().translation();
        float t;
        shape1->segment().closest_point(sphere_center, t, closest_pt);
        float r = shape1->radius() + shape2->radius();
        sqr_dist = sphere_center.square_distance(closest_pt);
        return sqr_dist <= r * r;
    }
};

class CapsuleBackend : public NpBackend<CapsuleShape, CapsuleShape> {
public:
    bool intersect(const Shape1* shape1, const Shape1* shape2)
    {
        vec3 closest_pt1;
        vec3 closest_pt2;
        float sqr_dist;
        return intersect_impl(shape1, shape2, closest_pt1, closest_pt2, sqr_dist);
    }

    bool collide(const Shape1* shape1, const Shape1* shape2, NpContactPatch& patch)
    {
        static constexpr float DIST_EPSILON = 1e-6f;

        vec3 closest_pt1;
        vec3 closest_pt2;
        float sqr_dist;
        if (!intersect_impl(shape1, shape2, closest_pt1, closest_pt2, sqr_dist))
            return false;

        float dist = sqrtf(sqr_dist);
        auto pen_axis = (dist > DIST_EPSILON) ? (closest_pt2 - closest_pt1) / dist : vec3{1.f, 0.f, 0.f};
        auto p1 = closest_pt1 + pen_axis * shape1->radius();
        auto p2 = closest_pt2 - pen_axis * shape2->radius();
        patch.contacts.push_back({p1, p2, pen_axis});

        return true;
    }

private:
    bool intersect_impl(const Shape1* shape1, const Shape1* shape2, vec3& closest_pt1, vec3& closest_pt2, float& sqr_dist)
    {
        float t1;
        float t2;
        shape1->segment().closest_point(shape2->segment(), t1, t2, closest_pt1, closest_pt2);
        float r = shape1->radius() + shape2->radius();
        sqr_dist = closest_pt1.square_distance(closest_pt2);
        return sqr_dist <= r * r;
    }
};

} // slope
