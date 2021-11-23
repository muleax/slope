#pragma once
#include "slope/math/vector3.hpp"

namespace slope {

class DebugDrawer {
public:
    virtual ~DebugDrawer() = default;

    virtual void draw_line(const Vec3& a, const Vec3& b, const Vec3& color) = 0;
    virtual void clear() = 0;
};

} // slope
