#pragma once
#include "slope/collision/narrowphase/narrowphase_backend.hpp"
#include "slope/core/array.hpp"
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

    void reset();
    void merge(const NarrowphaseStats& other);
};

class Narrowphase : public ConfigHolder<NarrowphaseConfig> {
public:
    Narrowphase();

    template<class Backend>
    void        add_backend();
    void        remove_all_backends();

    bool        intersect(const CollisionShape* shape1, const CollisionShape* shape2);
    bool        collide(const CollisionShape* shape1, const CollisionShape* shape2, NpContactPatch& patch);

    void        reset_stats();
    void        finalize_stats();
    const auto& stats() const { return m_stats; }

private:
    using Backends = Vector<std::unique_ptr<INpBackendWrapper>>;
    using BackendMap = Array<Array<INpBackendWrapper*, (int)ShapeKind::Count>, (int)ShapeKind::Count>;

    INpBackendWrapper* find_backend(const CollisionShape* shape1, const CollisionShape* shape2);
    void remove_backend(int type1, int type2);

    void on_config_update(const NarrowphaseConfig& prev_config) override;

    NpContext           m_context;
    NpNullBackend       m_null_backend;
    Backends            m_backends;
    BackendMap          m_backend_map;

    NarrowphaseStats    m_stats;
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
