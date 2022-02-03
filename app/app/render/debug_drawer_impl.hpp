#pragma once
#include "app/render/line.hpp"
#include "slope/debug/debug_drawer.hpp"
#include "slope/containers/vector.hpp"

namespace slope::app {

class DebugDrawerImpl : public DebugDrawer {
public:
    void draw_line(const vec3& a, const vec3& b, const vec3& color) final {
        m_lines_points.push_back({a, color});
        m_lines_points.push_back({b, color});
    }

    void clear() final { m_lines_points.clear(); }

    const Vector<LinePoint>& lines_points() const { return m_lines_points; }

private:
    Vector<LinePoint> m_lines_points;
};

} // slope::app
