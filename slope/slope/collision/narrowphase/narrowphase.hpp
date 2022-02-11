#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/core/array.hpp"
#include "slope/core/stats_holder.hpp"
#include "slope/core/config_holder.hpp"
#include "slope/debug/log.hpp"
#include <memory>

namespace slope {

struct NarrowphaseConfig {
    GJKConfig gjk_config;
    EPAConfig epa_config;
};

struct NarrowphaseStats {
    uint32_t test_count = 0;
    uint32_t collision_count = 0;
    uint32_t contact_count = 0;

    GJKStats gjk_stats;
    EPAStats epa_stats;
    SATStats sat_stats;

    void reset()
    {
        test_count = 0;
        collision_count = 0;
        contact_count = 0;

        gjk_stats.reset();
        epa_stats.reset();
        sat_stats.reset();
    }

    void merge(const NarrowphaseStats& other)
    {
        test_count += other.test_count;
        collision_count += other.collision_count;
        contact_count += other.contact_count;

        gjk_stats.merge(other.gjk_stats);
        epa_stats.merge(other.epa_stats);
        sat_stats.merge(other.sat_stats);
    }
};

class Narrowphase : public ConfigHolder<NarrowphaseConfig>, public StatsHolder<NarrowphaseStats> {
public:
    Narrowphase();

    template<class Backend>
    void        add_backend();
    void        remove_all_backends();

    bool        intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    bool        collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch);

    void        reset_stats() override
    {
        StatsHolder::reset_stats();
        m_context.gjk_solver.reset_stats();
        m_context.epa_solver.reset_stats();
        m_context.sat_solver.reset_stats();
    }

    void        finalize_stats() override
    {
        m_stats.gjk_stats = m_context.gjk_solver.stats();
        m_stats.epa_stats = m_context.epa_solver.stats();
        m_stats.sat_stats = m_context.sat_solver.stats();
    }

private:
    INpBackendWrapper* find_backend(const CollisionShape* shape1, const CollisionShape* shape2);
    void remove_backend(int type1, int type2);

    NpContext m_context;
    NpNullBackend m_null_backend;
    Vector<std::unique_ptr<INpBackendWrapper>> m_backends;
    Array<Array<INpBackendWrapper*, (int)ShapeKind::Count>, (int)ShapeKind::Count> m_backend_map;
};

template<class Backend>
void Narrowphase::add_backend()
{
    auto kind1 = static_cast<int>(Backend::Shape1::s_kind());
    auto kind2 = static_cast<int>(Backend::Shape2::s_kind());

    remove_backend(kind1, kind2);

    m_backends.push_back(std::make_unique<NpBackendWrapper<Backend>>(&m_context));
    m_backend_map[kind1][kind2] = m_backends.back().get();

    if (kind1 != kind2) {
        m_backends.push_back(std::make_unique<NpBackendWrapper<Backend, true>>(&m_context));
        m_backend_map[kind2][kind1] = m_backends.back().get();
    }
}

inline INpBackendWrapper* Narrowphase::find_backend(const CollisionShape* shape1, const CollisionShape* shape2)
{
    return m_backend_map[(int)shape1->kind()][(int)shape2->kind()];
}

inline bool Narrowphase::intersect(const CollisionShape* shape1, const CollisionShape* shape2)
{
    bool result = find_backend(shape1, shape2)->intersect(shape1, shape2);
    m_stats.test_count++;
    m_stats.collision_count += static_cast<int>(result);
    return result;
}

inline bool Narrowphase::collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch)
{
    bool result = find_backend(shape1, shape2)->collide(shape1, shape2, patch);
    m_stats.test_count++;
    m_stats.collision_count += static_cast<int>(result);
    m_stats.contact_count += patch.contacts.size();
    return result;
}

} // slope
