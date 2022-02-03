#pragma once
#include "slope/math/vector3.hpp"

namespace slope {

class DebugDrawer {
public:
    virtual ~DebugDrawer() = default;

    virtual void draw_line(const vec3& a, const vec3& b, const vec3& color) = 0;
    virtual void clear() = 0;
};

} // slope
