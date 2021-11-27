#pragma once
#include "slope/collision/narrowphase_backend.hpp"
#include "slope/containers/array.hpp"
#include <optional>

namespace slope {

template <class... Backends>
class NarrowphaseImpl {
public:
    NarrowphaseImpl();

    bool intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    void generate_contacts(ContactManifold& manifold);

    NpBackendHint backend_hint() const { return m_backend_hint; }
    void set_backend_hint(NpBackendHint value) { m_backend_hint = value; }

private:
    struct BackendRoutines {
        using IntersectFn = bool (NpContext& ctx, const CollisionShape*, const CollisionShape*);
        using GenerateFn = void (NpContext& ctx, ContactManifold& manifold, const CollisionShape*, const CollisionShape*);

        IntersectFn*    intersect = NullBackend::intersect;
        GenerateFn*     generate_contacts = NullBackend::generate_contacts;
        bool            initialized = false;

        template <class Backend>
        void initialize() {
            intersect = Backend::intersect;
            generate_contacts = Backend::generate_contacts;
            initialized = true;
        }
    };

    NpContext m_context;
    NpBackendHint m_backend_hint = NpBackendHint::GJK_EPA;

    const CollisionShape* m_shape1 = nullptr;
    const CollisionShape* m_shape2 = nullptr;
    BackendRoutines* m_current_backend = nullptr;

    Array<Array<Array<BackendRoutines, (int)NpBackendHint::Count>, (int)ShapeType::Count>, (int)ShapeType::Count> m_backends;
};

template <class... Backends>
bool NarrowphaseImpl<Backends...>::intersect(const CollisionShape* shape1, const CollisionShape* shape2)
{
    m_shape1 = shape1;
    m_shape2 = shape2;
    m_current_backend = &m_backends[(int)shape1->type()][(int)shape2->type()][(int)m_backend_hint];
    return m_current_backend->intersect(m_context, shape1, shape2);
}

template <class... Backends>
void NarrowphaseImpl<Backends...>::generate_contacts(ContactManifold& manifold)
{
    return m_current_backend->generate_contacts(m_context, manifold, m_shape1, m_shape2);
}

template <class... Backends>
NarrowphaseImpl<Backends...>::NarrowphaseImpl()
{
    ((m_backends[(int)Backends::Shape2::Type][(int)Backends::Shape1::Type][(int)Backends::Hint_]. template initialize<NpBackendWrapper<Backends, true>>()), ...);
    ((m_backends[(int)Backends::Shape1::Type][(int)Backends::Shape2::Type][(int)Backends::Hint_]. template initialize<NpBackendWrapper<Backends, false>>()), ...);

    // fallback to GJK/EPA
    for (auto& per_shape1 : m_backends)
        for (auto& per_shape2 : per_shape1)
            for (auto& backend : per_shape2)
                if (!backend.initialized)
                    backend = per_shape2[(int)NpBackendHint::GJK_EPA];

}

using Narrowphase = NarrowphaseImpl<
    ConvexPolyhedronBackend<NpBackendHint::GJK_EPA>,
    ConvexPolyhedronBackend<NpBackendHint::SAT>,
    ConvexPolyhedronSphereBackend,
    SphereBackend>;

} // slope
