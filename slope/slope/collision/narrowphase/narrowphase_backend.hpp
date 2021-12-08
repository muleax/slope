#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/gjk.hpp"
#include "slope/collision/sat.hpp"
#include "slope/collision/epa.hpp"
#include "slope/collision/face_clipper.hpp"
#include "slope/containers/vector.hpp"
#include "slope/collision/primitives.hpp"

namespace slope {

struct NpContactPatch {
    Vector<ContactGeom> contacts;
    bool inverted_order = false;

    void reset()
    {
        contacts.clear();
        inverted_order = false;
    }

    void invert_input_order() {
        inverted_order = !inverted_order;
    }

    void normalize_order()
    {
        if (inverted_order) {
            for (auto& geom : contacts) {
                std::swap(geom.p1, geom.p2);
                geom.normal = -geom.normal;
            }
            inverted_order = false;
        }
    }
};

struct NpContext {
    GJKSolver gjk_solver;
    EPASolver epa_solver;
    SATSolver sat_solver;

    FaceClipper face_clipper;
    Vector<Vec3> support_face[2];
    Vector<Vec3> clipped_face;
};

// Derive specialization in order to support collision pair
template <class A, class B>
class NpBackend {
public:
    using Shape1 = A;
    using Shape2 = B;

    void        set_ctx(NpContext* context) { m_ctx = context; }
    NpContext*  ctx() const { return m_ctx; }

    // Expected interface:
    // bool intersect(const Shape1* shape1, const Shape2* shape2);
    // bool collide(const Shape1* shape1, const Shape2* shape2, NpContactPatch& patch);

private:
    NpContext* m_ctx = nullptr;
};

class INpBackendWrapper {
public:
    virtual ~INpBackendWrapper() = default;
    virtual bool intersect(const CollisionShape* shape1, const CollisionShape* shape2) = 0;
    virtual bool collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch) = 0;
};

class NpNullBackend : public INpBackendWrapper {
    bool intersect(const CollisionShape* shape1, const CollisionShape* shape2) final { return false; }
    bool collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch) final { return false; }
};

template <class Backend, bool Inverted = false>
class NpBackendWrapper : public INpBackendWrapper {
public:
    using Shape1 = typename Backend::Shape1;
    using Shape2 = typename Backend::Shape2;

    explicit NpBackendWrapper(NpContext* context)
    {
        m_backend.set_ctx(context);
    }

    bool intersect(const CollisionShape* shape1, const CollisionShape* shape2) final
    {
        if constexpr (Inverted) {
            std::swap(shape1, shape2);
        }

        return m_backend.intersect((const Shape1*)shape1, (const Shape2*)shape2);
    }

    bool collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch) final
    {
        if constexpr (Inverted) {
            std::swap(shape1, shape2);
            patch.invert_input_order();
        }

        return m_backend.collide((const Shape1*)shape1, (const Shape2*)shape2, patch);
    }

private:
    Backend m_backend;
};

} // slope
