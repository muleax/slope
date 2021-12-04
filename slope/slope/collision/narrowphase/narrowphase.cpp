#include "slope/collision/narrowphase/narrowphase.hpp"

namespace slope {

Narrowphase::Narrowphase()
{
    reset_all_backends();
}

void Narrowphase::reset_all_backends()
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

} // slope
