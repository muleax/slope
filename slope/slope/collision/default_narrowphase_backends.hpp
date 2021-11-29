#pragma once
#include "slope/collision/narrowphase_backend.hpp"
#include "slope/collision/convex_polyhedron_shape.hpp"
#include "slope/collision/sphere_shape.hpp"

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
