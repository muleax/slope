#pragma once
#include <optional>

namespace slope {

template <class T>
class ConfigHolder {
public:
    const T&    config() const { return m_config; }
    void        update_config(const T& new_config);

protected:
    virtual void on_config_update(const T& prev_config) {}

private:
    T m_config;
};

template <class T>
inline void ConfigHolder<T>::update_config(const T& new_config)
{
    T prev_config = m_config;
    m_config = new_config;
    on_config_update(prev_config);
}

} // slope
