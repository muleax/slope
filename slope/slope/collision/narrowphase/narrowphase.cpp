#include "slope/collision/narrowphase/narrowphase.hpp"

namespace slope {

void NarrowphaseStats::reset()
{
    test_count = 0;
    collision_count = 0;
    contact_count = 0;

    gjk_stats.reset();
    epa_stats.reset();
    sat_stats.reset();
}

void NarrowphaseStats::merge(const NarrowphaseStats& other)
{
    test_count += other.test_count;
    collision_count += other.collision_count;
    contact_count += other.contact_count;

    gjk_stats.merge(other.gjk_stats);
    epa_stats.merge(other.epa_stats);
    sat_stats.merge(other.sat_stats);
}

Narrowphase::Narrowphase()
{
    remove_all_backends();
}

void Narrowphase::remove_all_backends()
{
    for (auto& container : m_backend_map)
        container.fill(&m_null_backend);
}

void Narrowphase::remove_backend(int type1, int type2)
{
    INpBackendWrapper* backend = m_backend_map[type1][type2];
    if (backend == &m_null_backend)
        return;

    m_backends.erase(std::find_if(m_backends.begin(), m_backends.end(),
                                  [&backend](auto& v) { return v.get() == backend; }));
    m_backend_map[type1][type2] = &m_null_backend;

    if (type1 != type2) {
        INpBackendWrapper* inv_backend = m_backend_map[type2][type1];
        m_backend_map[type2][type1] = &m_null_backend;
    }
}

void Narrowphase::reset_stats()
{
    m_stats.reset();
    m_context.gjk_solver.reset_stats();
    m_context.epa_solver.reset_stats();
    m_context.sat_solver.reset_stats();
}

void Narrowphase::finalize_stats()
{
    m_stats.gjk_stats = m_context.gjk_solver.stats();
    m_stats.epa_stats = m_context.epa_solver.stats();
    m_stats.sat_stats = m_context.sat_solver.stats();
}

} // slope
