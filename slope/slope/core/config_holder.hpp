#pragma once
#include <optional>

namespace slope {

template <class T>
class ConfigHolder {
public:
    const T&        config() const { return m_config; }
    virtual void    update_config(const T& new_config) { m_config = new_config; }

protected:
    T m_config;
};

} // slope
