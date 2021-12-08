#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/containers/array.hpp"
#include <memory>

namespace slope {

class Narrowphase {
public:
    Narrowphase();

    template<class Backend>
    void        add_backend();
    void        reset_all_backends();

    bool        intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    bool        collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch);

    GJKSolver&  gjk_solver() { return m_context.gjk_solver; }
    EPASolver&  epa_solver() { return m_context.epa_solver; }
    SATSolver&  sat_solver() { return m_context.sat_solver; }

private:
    INpBackendWrapper* find_backend(const CollisionShape* shape1, const CollisionShape* shape2);
    void remove_backend(int type1, int type2);

    NpContext m_context;
    NpNullBackend m_null_backend;
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

inline INpBackendWrapper* Narrowphase::find_backend(const CollisionShape* shape1, const CollisionShape* shape2)
{
    return m_backend_map[(int)shape1->type()][(int)shape2->type()];
}

inline bool Narrowphase::intersect(const CollisionShape* shape1, const CollisionShape* shape2)
{
    return find_backend(shape1, shape2)->intersect(shape1, shape2);
}

inline bool Narrowphase::collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch)
{
    return find_backend(shape1, shape2)->collide(shape1, shape2, patch);
}

} // slope
