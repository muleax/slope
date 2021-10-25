#pragma once
#include <cstdint>
#include <limits>

namespace slope::app {

class Entity {
public:
    using Id = uint32_t;
    static constexpr Id INVALID_ID = std::numeric_limits<Id>::max();

    Entity() = default;

    explicit Entity(Id id) : m_id(id) {}

    Id id() const { return m_id; }

    friend bool operator== (const Entity& a, const Entity& b) { return a.id() == b.id(); }

private:
    Id m_id = INVALID_ID;
};

} // slope::app

namespace std
{
template<> struct hash<slope::app::Entity>
{
    std::size_t operator()(slope::app::Entity const& e) const noexcept
    {
        return e.id();
    }
};
}