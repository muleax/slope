#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/collision/shape/convex_polyhedron_shape.hpp"
#include "slope/collision/shape/sphere_shape.hpp"
#include "slope/collision/shape/capsule_shape.hpp"

namespace slope {

class SATConvexPolyhedronBackend : public NpBackend<ConvexPolyhedronShape, ConvexPolyhedronShape> {
public:
    bool intersect();
    void generate_contacts(ContactManifold& manifold);

private:
    std::optional<Vec3> m_min_pen_axis;
};

class GJKConvexPolyhedronBackend : public NpBackend<ConvexPolyhedronShape, ConvexPolyhedronShape> {
public:
    bool intersect();
    void generate_contacts(ContactManifold& manifold);
};

class ConvexPolyhedronSphereBackend : public NpBackend<ConvexPolyhedronShape, SphereShape> {
public:
    bool intersect();
    void generate_contacts(ContactManifold& manifold);
};

class ConvexPolyhedronCapsuleBackend : public NpBackend<ConvexPolyhedronShape, CapsuleShape> {
public:
    bool intersect()
    {
        return context()->gjk_solver.intersect(shape1(), shape2());
    }

    void generate_contacts(ContactManifold& manifold);
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

    void generate_contacts(ContactManifold& manifold)
    {
        Vec3 pen_dir = m_p2 - m_p1;
        float pen_axis_len = pen_dir.length();
        Vec3 pen_axis = (pen_axis_len > 1e-6f) ? pen_dir / pen_axis_len : Vec3{1.f, 0.f, 0.f};

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

    void generate_contacts(ContactManifold& manifold)
    {
        Vec3 pen_dir = m_p2 - m_p1;
        float pen_axis_len = pen_dir.length();
        Vec3 pen_axis = (pen_axis_len > 1e-6f) ? pen_dir / pen_axis_len : Vec3{1.f, 0.f, 0.f};

        manifold.add_contact({m_p1 + pen_axis * shape1()->radius(), m_p2 - pen_axis * shape2()->radius(), pen_axis});
    }

private:
    Vec3 m_p1;
    Vec3 m_p2;
};

class SphereBackend : public NpBackend<SphereShape, SphereShape> {
public:
    bool intersect();
    void generate_contacts(ContactManifold& manifold);

private:
    float m_dist_sqr = 0.f;
};

inline bool SATConvexPolyhedronBackend::intersect()
{
    m_min_pen_axis = context()->sat_solver.find_penetration_axis(shape1(), shape2());
    return m_min_pen_axis.has_value();
}

inline bool GJKConvexPolyhedronBackend::intersect()
{
    return context()->gjk_solver.intersect(shape1(), shape2());
}

inline bool ConvexPolyhedronSphereBackend::intersect()
{
    return context()->gjk_solver.intersect(shape1(), shape2());
}

} // slope
