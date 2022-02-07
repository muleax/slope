#pragma once
#include "slope/core/unordered_map.hpp"
#include <tuple>
#include <cstdint>

namespace slope {

class PairCacheKey {
public:
    using Data = std::pair<const void*, const void*>;

    PairCacheKey(const void* a, const void* b) : m_data(a < b ? Data{a, b} : Data{b, a}) {}

    size_t hash() const
    {
        auto a = reinterpret_cast<size_t>(m_data.first);
        auto b = reinterpret_cast<size_t>(m_data.second);
        return a ^ (b + 0x9e3779b9 + (a << 6) + (a >> 2));
    }

    bool operator==(const PairCacheKey& other) const { return m_data == other.m_data; }

private:
    Data m_data;
};

} // slope

namespace std {
template<> struct hash<slope::PairCacheKey>
{
    std::size_t operator()(const slope::PairCacheKey& key) const noexcept
    {
        return key.hash();
    }
};
} // std

namespace slope {

template <class T>
using PairCache = UnorderedMap<PairCacheKey, T>;

} // slope
