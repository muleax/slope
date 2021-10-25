#pragma once
#include <cstdint>

namespace slope::app {

using RenderHandle = uint32_t;

struct ShaderLayout {
    static constexpr uint32_t   ATTR_POSITION = 0;
    static constexpr uint32_t   ATTR_NORMAL = 1;
    static constexpr uint32_t   ATTR_COLOR = 2;
    static constexpr uint32_t   ATTR_TEX_COORDS = 3;
    static constexpr uint32_t   ATTR_MVP_0 = 4;
    static constexpr uint32_t   ATTR_MVP_1 = 5;
    static constexpr uint32_t   ATTR_MVP_2 = 6;
    static constexpr uint32_t   ATTR_MVP_3 = 7;
};

} // slope::app