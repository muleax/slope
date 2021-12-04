#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/containers/array.hpp"
#include <memory>

namespace slope {

class Narrowphase {
public:
    Narrowphase();

    template<class Backend>
    void                add_backend();
    void                reset_all_backends();

    bool                intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    std::optional<Vec3> get_penetration_axis();
    void                generate_contacts(ContactManifold& manifold);

    bool                collide(ContactManifold& manifold, const CollisionShape* shape1, const CollisionShape* shape2);

    GJKSolver&          gjk_solver() { return m_context.gjk_solver; }
    EPASolver&          epa_solver() { return m_context.epa_solver; }
    SATSolver&          sat_solver() { return m_context.sat_solver; }

private:
    void remove_backend(int type1, int type2);

    NpContext m_context;

    NpNullBackend m_null_backend;
    INpBackendWrapper* m_current_backend = nullptr;

    Vector<std::unique_ptr<INpBackendWrapper>> m_backends;
    Array<Array<INpBackendWrapper*, (int)ShapeType::Count>, (int)ShapeType::Count> m_backend_map;
};

template<class Backend>
void Narrowphase::add_backend()
{
    auto type1 = static_cast<int>(Backend::Shape1::Type);
    auto type2 = static_cast<int>(Backend::Shape2::Type);

    remove_backend(type1, type2);

    m_backends.push_back(std::make_unique<NpBackendWrapper<Backend>>(&m_context));
    m_backend_map[type1][type2] = m_backends.back().get();

    if (type1 != type2) {
        m_backends.push_back(std::make_unique<NpBackendWrapper<Backend, true>>(&m_context));
        m_backend_map[type2][type1] = m_backends.back().get();
    }
}

inline bool Narrowphase::intersect(const CollisionShape* shape1, const CollisionShape* shape2)
{
    m_current_backend = m_backend_map[static_cast<int>(shape1->type())][static_cast<int>(shape2->type())];
    return m_current_backend->intersect(shape1, shape2);
}

inline std::optional<Vec3> Narrowphase::get_penetration_axis()
{
    return m_current_backend->get_penetration_axis();
}

inline void Narrowphase::generate_contacts(ContactManifold& manifold)
{
    manifold.begin_update();
    m_current_backend->generate_contacts(manifold);
    manifold.end_update();
}

inline bool Narrowphase::collide(ContactManifold& manifold, const CollisionShape* shape1, const CollisionShape* shape2)
{
    if (intersect(shape1, shape2)) {
        generate_contacts(manifold);
        return true;
    }

    return false;
}

} // slope
