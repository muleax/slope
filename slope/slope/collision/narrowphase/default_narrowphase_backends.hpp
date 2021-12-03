#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/convex_polyhedron_shape.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/capsule_shape.hpp"

namespace slope {

template <class A, class B>
class GJKBackend : public NpBackend<A, B> {
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
};

class GJKConvexPolyhedronBackend : public GJKBackend<ConvexPolyhedronShape, ConvexPolyhedronShape> {
public:
    void generate_contacts(ContactManifold& manifold);
};

class ConvexPolyhedronSphereBackend : public GJKBackend<ConvexPolyhedronShape, SphereShape> {
public:
    void generate_contacts(ContactManifold& manifold);
};

class ConvexPolyhedronCapsuleBackend : public GJKBackend<ConvexPolyhedronShape, CapsuleShape> {
public:
    void generate_contacts(ContactManifold& manifold);
};

class SATConvexPolyhedronBackend : public NpBackend<ConvexPolyhedronShape, ConvexPolyhedronShape> {
public:
    bool intersect()
    {
        m_min_pen_axis = context()->sat_solver.find_penetration_axis(shape1(), shape2());
        return m_min_pen_axis.has_value();
    }

    std::optional<Vec3> get_penetration_axis() { return m_min_pen_axis; }
    void                generate_contacts(ContactManifold& manifold);

private:
    std::optional<Vec3> m_min_pen_axis;
};

class CapsuleSphereBackend : public NpBackend<CapsuleShape, SphereShape> {
public:
    bool intersect()
    {
        float t1;
        float t2;
        auto& sp = shape2()->transform().translation();
        // TODO: optimize
        float sqr_dist = shape1()->segment().closest_point({sp, sp}, t1, t2, m_p1, m_p2);
        float r = shape1()->radius() + shape2()->radius();
        return sqr_dist <= r * r;
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
        manifold.add_contact({m_p1 + pen_axis * shape1()->radius(), m_p2 - pen_axis * shape2()->radius(), pen_axis});
    }

private:
    Vec3 m_p1;
    Vec3 m_p2;
};

class CapsuleBackend : public NpBackend<CapsuleShape, CapsuleShape> {
public:
    bool intersect()
    {
        float t1;
        float t2;
        float sqr_dist = shape1()->segment().closest_point(shape2()->segment(), t1, t2, m_p1, m_p2);
        float r = shape1()->radius() + shape2()->radius();
        return sqr_dist <= r * r;
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
        manifold.add_contact({m_p1 + pen_axis * shape1()->radius(), m_p2 - pen_axis * shape2()->radius(), pen_axis});
    }

private:
    Vec3 m_p1;
    Vec3 m_p2;
};

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
