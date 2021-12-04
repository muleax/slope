#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/polyhedron_shape.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/box_shape.hpp"

namespace slope {

template <class A, class B>
class PolyhedraSphereBackend : public NpBackend<A, B> {
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
        // TODO: optimize
        auto* ctx = this->context();
        Vec3 face_normal;
        ctx->support_face[0].clear();
        this->shape1()->get_support_face(*pen_axis, ctx->support_face[0], face_normal);

        auto p2 = this->shape2()->support_normalized(-*pen_axis, 0.f);
        auto t = Plane(face_normal, ctx->support_face[0][0]).intersect_ray(p2, *pen_axis);
        if (t) {
            auto p1 = p2 + *t * *pen_axis;
            manifold.add_contact({p1, p2, *pen_axis});
        }
    }
};

using ConvexPolyhedronSphereBackend = PolyhedraSphereBackend<PolyhedronShape, SphereShape>;
using BoxSphereBackend = PolyhedraSphereBackend<BoxShape, SphereShape>;

class SphereBackend : public NpBackend<SphereShape, SphereShape> {
public:
    bool intersect()
    {
        m_p1 = shape1()->transform().translation();
        m_p2 = shape2()->transform().translation();
        m_dist_sqr = m_p1.square_distance(m_p2);
        float contact_dist = shape1()->radius() + shape2()->radius();
        return m_dist_sqr <= contact_dist * contact_dist;
    }

    std::optional<Vec3> get_penetration_axis()
    {
        static constexpr float DIST_EPSILON = 1e-6f;
        float dist = sqrtf(m_dist_sqr);
        return dist > DIST_EPSILON ? (m_p2 - m_p1) / dist : Vec3{1.f, 0.f, 0.f};
    }

    void generate_contacts(ContactManifold& manifold)
    {
        auto pen_axis = *get_penetration_axis();
        manifold.add_contact({m_p1 + pen_axis * shape1()->radius(), m_p2 - pen_axis * shape2()->radius(), pen_axis});
    }

private:
    Vec3 m_p1;
    Vec3 m_p2;
    float m_dist_sqr = 0.f;
};

} // slope
