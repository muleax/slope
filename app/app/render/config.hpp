#pragma once
#include <cstdint>

namespace slope::app {

using RenderHandle = uint32_t;

struct ShaderAttributeLayout {
    static constexpr uint32_t   POSITION = 0;
    static constexpr uint32_t   NORMAL = 1;
    static constexpr uint32_t   COLOR = 2;
    static constexpr uint32_t   TEX_COORDS = 3;
    static constexpr uint32_t   MODEL_0 = 4;
    static constexpr uint32_t   MODEL_1 = 5;
    static constexpr uint32_t   MODEL_2 = 6;
    static constexpr uint32_t   MODEL_3 = 7;
};

struct ShaderUniformLayout {
    static constexpr char VIEW_PROJECTION[] = "view_projection";
    static constexpr char LIGHT_POSITION[] = "light_position";
    static constexpr char AMBIENT_STRENGTH[] = "ambient_strength";
};

} // slope::app