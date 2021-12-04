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
        if (!pen_axis)
            return;

        // TODO: robust solution for imprecise penetration axis
        auto* ctx = this->context();
        auto& support_face = ctx->support_face[0];
        support_face.clear();

        Vec3 support_normal;
        this->shape1()->get_support_face(*pen_axis, support_face, support_normal);

        auto segment = ctx->face_clipper.clip_segment_by_convex_prism(
            this->shape2()->segment(), support_face, *pen_axis);

        for (int i = 0; i < 2; i++) {
            auto clipped_pt = segment[i] - *pen_axis * this->shape2()->radius();
            auto t = Plane(support_normal, support_face[0]).intersect_ray(clipped_pt, *pen_axis);
            if (t) {
                auto p1 = clipped_pt + *t * *pen_axis;
                manifold.add_contact({p1, clipped_pt, *pen_axis});
            }
        }
    }
};

using ConvexPolyhedronCapsuleBackend = PolyhedraCapsuleBackend<PolyhedronShape, CapsuleShape>;
using BoxCapsuleBackend = PolyhedraCapsuleBackend<BoxShape, CapsuleShape>;

class CapsuleSphereBackend : public NpBackend<CapsuleShape, SphereShape> {
public:
    bool intersect()
    {
        auto& sphere_center = shape2()->transform().translation();
        float t;
        shape1()->segment().closest_point(sphere_center, t, m_p);
        float r = shape1()->radius() + shape2()->radius();
        return sphere_center.square_distance(m_p) <= r * r;
    }

    std::optional<Vec3> get_penetration_axis()
    {
        Vec3 pen_dir = shape2()->transform().translation() - m_p;
        float pen_axis_len = pen_dir.length();
        return (pen_axis_len > 1e-6f) ? pen_dir / pen_axis_len : Vec3{1.f, 0.f, 0.f};
    }

    void generate_contacts(ContactManifold& manifold)
    {
        auto pen_axis = *get_penetration_axis();
        auto p1 = m_p + pen_axis * shape1()->radius();
        auto p2 = shape2()->transform().translation() - pen_axis * shape2()->radius();
        manifold.add_contact({p1, p2, pen_axis});
    }

private:
    Vec3 m_p;
};

class CapsuleBackend : public NpBackend<CapsuleShape, CapsuleShape> {
public:
    bool intersect()
    {
        float t1;
        float t2;
        shape1()->segment().closest_point(shape2()->segment(), t1, t2, m_p1, m_p2);
        float r = shape1()->radius() + shape2()->radius();
        return m_p1.square_distance(m_p2) <= r * r;
    }

    std::optional<Vec3> get_penetration_axis()
    {
        Vec3 pen_dir = m_p2 - m_p1;
        float pen_axis_len = pen_dir.length();
        return (pen_axis_len > 1e-6f) ? pen_dir / pen_axis_len : Vec3{1.f, 0.f, 0.f};
    }

    void generate_contacts(ContactManifold& manifold)
    {
        auto pen_axis = *get_penetration_axis();
        auto p1 = m_p1 + pen_axis * shape1()->radius();
        auto p2 = m_p2 - pen_axis * shape2()->radius();
        manifold.add_contact({p1, p2, pen_axis});
    }

private:
    Vec3 m_p1;
    Vec3 m_p2;
};

} // slope
