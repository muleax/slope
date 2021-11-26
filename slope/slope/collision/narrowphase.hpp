#pragma once
#include "slope/collision/collision_shape.hpp"
#include "slope/collision/convex_polyhedron_shape.hpp"
#include "slope/collision/gjk.hpp"
#include "slope/collision/sat.hpp"
#include "slope/collision/epa.hpp"
#include "slope/collision/face_clipper.hpp"
#include "slope/collision/contact_manifold.hpp"
#include <optional>

namespace slope {

class Narrowphase {
public:
    enum class Backend {
        SAT,
        GJK_EPA
    };

    std::optional<Vec3> find_penetration_axis(const CollisionShape* shape1, const CollisionShape* shape2);

    void generate_contacts(
        ContactManifold& manifold, const Vec3& pen_axis,
        const CollisionShape* shape1, const CollisionShape* shape2);

    void generate_contacts(
        ContactManifold& manifold, const Vec3& pen_axis,
        const ConvexPolyhedronShape* shape1, const ConvexPolyhedronShape* shape2);

    Backend preferred_backend() const { return m_preferred_backend; }
    void set_preferred_backend(Backend value) { m_preferred_backend = value; }

private:
    GJKSolver m_gjk_solver;
    EPASolver m_epa_solver;
    SATSolver m_sat_solver;

    FaceClipper m_face_clipper;
    Vector<Vec3> m_support_face[2];
    Vector<Vec3> m_clipped_face;

    Backend m_preferred_backend = Backend::GJK_EPA;
};

} // slope
