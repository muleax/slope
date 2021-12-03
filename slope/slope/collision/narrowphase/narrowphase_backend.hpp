#pragma once
#include "slope/collision/shape/collision_shape.hpp"
#include "slope/collision/gjk.hpp"
#include "slope/collision/sat.hpp"
#include "slope/collision/epa.hpp"
#include "slope/collision/face_clipper.hpp"
#include "slope/collision/contact_manifold.hpp"

namespace slope {

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

    void            set_context(NpContext* context) { m_context = context; }
    NpContext*      context() const { return m_context; }

    void            set_shapes(const Shape1* shape1, const Shape2* shape2) { m_shape1 = shape1; m_shape2 = shape2; }
    const Shape1*   shape1() const { return m_shape1; }
    const Shape2*   shape2() const { return m_shape2; }

    // Expected interface:
    // bool                 intersect(const Shape1* shape1, const Shape2* shape2);
    // std::optional<Vec3>  get_penetration_axis();
    // void                 generate_contacts(ContactManifold& manifold, const Shape1* shape1, const Shape2* shape2);

private:
    NpContext*      m_context = nullptr;
    const Shape1*   m_shape1 = nullptr;
    const Shape2*   m_shape2 = nullptr;
};

class INpBackendWrapper {
public:
    virtual ~INpBackendWrapper() = default;

    virtual bool                intersect(const CollisionShape* shape1, const CollisionShape* shape2) = 0;
    virtual void                generate_contacts(ContactManifold& manifold) = 0;
    virtual std::optional<Vec3> get_penetration_axis() = 0;

};

class NpNullBackend : public INpBackendWrapper {
    bool                intersect(const CollisionShape* shape1, const CollisionShape* shape2) final { return false; }
    std::optional<Vec3> get_penetration_axis() final { return std::nullopt; }
    void                generate_contacts(ContactManifold& manifold) final {}
};

template <class Backend, bool Inverted = false>
class NpBackendWrapper : public INpBackendWrapper {
public:
    using Shape1 = typename Backend::Shape1;
    using Shape2 = typename Backend::Shape2;

    explicit NpBackendWrapper(NpContext* context)
    {
        m_backend.set_context(context);
    }

    bool intersect(const CollisionShape* shape1, const CollisionShape* shape2) final
    {
        if constexpr (Inverted)
            std::swap(shape1, shape2);

        m_backend.set_shapes(static_cast<const Shape1*>(shape1), static_cast<const Shape2*>(shape2));
        return m_backend.intersect();
    }

    void generate_contacts(ContactManifold& manifold) final
    {
        if constexpr (Inverted)
            manifold.invert_input_order();

        m_backend.generate_contacts(manifold);
    }

    std::optional<Vec3> get_penetration_axis() final
    {
        return m_backend.get_penetration_axis();
    }

private:
    Backend m_backend;
};

} // slope
