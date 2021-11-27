#pragma once
#include "slope/collision/narrowphase_backend.hpp"
#include "slope/containers/array.hpp"
#include <optional>

namespace slope {

template <class... Shapes>
class NarrowphaseImpl {
public:
    NarrowphaseImpl();

    bool intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    void generate_contacts(ContactManifold& manifold);

    NpBackendHint backend_hint() const { return m_backend_hint; }
    void set_backend_hint(NpBackendHint value) { m_backend_hint = value; }

private:
    struct BackendRoutines {
        bool (*intersect)(NpContext& ctx, const CollisionShape*, const CollisionShape*) = nullptr;
        void (*generate_contacts)(NpContext& ctx, ContactManifold& manifold, const CollisionShape*, const CollisionShape*) = nullptr;
    };

    template <class Shape1, NpBackendHint Hint>
    void init_backends();

    NpContext m_context;
    NpBackendHint m_backend_hint = NpBackendHint::GJK_EPA;

    const CollisionShape* m_shape1 = nullptr;
    const CollisionShape* m_shape2 = nullptr;
    BackendRoutines* m_current_backend = nullptr;

    BackendRoutines m_backends[(int)ShapeType::Count][(int)ShapeType::Count][(int)NpBackendHint::Count];
};

template <class... Shapes>
bool NarrowphaseImpl<Shapes...>::intersect(const CollisionShape* shape1, const CollisionShape* shape2)
{
    m_shape1 = shape1;
    m_shape2 = shape2;
    m_current_backend = &m_backends[(int)shape1->type()][(int)shape2->type()][(int)m_backend_hint];
    return m_current_backend->intersect(m_context, shape1, shape2);
}

template <class... Shapes>
void NarrowphaseImpl<Shapes...>::generate_contacts(ContactManifold& manifold)
{
    return m_current_backend->generate_contacts(m_context, manifold, m_shape1, m_shape2);
}

template <class... Shapes>
template <class Shape1, NpBackendHint Hint>
void NarrowphaseImpl<Shapes...>::init_backends()
{
    ((m_backends[(int)Shape1::Type][(int)(Shapes::Type)][(int)Hint].intersect = NpBackendWrapper<Shape1, Shapes, Hint>::intersect), ...);
    ((m_backends[(int)Shape1::Type][(int)(Shapes::Type)][(int)Hint].generate_contacts = NpBackendWrapper<Shape1, Shapes, Hint>::generate_contacts), ...);
}

template <class... Shapes>
NarrowphaseImpl<Shapes...>::NarrowphaseImpl()
{
    (init_backends<Shapes, NpBackendHint::GJK_EPA>(), ...);
    (init_backends<Shapes, NpBackendHint::SAT>(), ...);
}

using Narrowphase = NarrowphaseImpl<ConvexPolyhedronShape, SphereShape>;

} // slope
