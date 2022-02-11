#pragma once

namespace slope {

template <class T>
class StatsHolder {
public:
    virtual void    reset_stats() { m_stats.reset(); }
    virtual void    finalize_stats() {}
    const T&        stats() const { return m_stats; }

protected:
    T m_stats;
};

} // slope
