#pragma once
#include "app/render/config.hpp"
#include "slope/math/math.hpp"
#include <memory>

namespace slope::app {

class ShaderProgram
{
public:
    using Location = int32_t;

    ShaderProgram(const char* vertex_shader_src, const char* fragment_shader_src);
    ~ShaderProgram();

    bool ready() const { return m_id != 0; }
    void use() const;

    Location attribute_location(const char* name) const;
    Location location(const char* name) const;

    void set(Location loc, bool value) const;
    void set(Location loc, int value) const;
    void set(Location loc, float value) const;
    void set(Location loc, const Vec2& value) const;
    void set(Location loc, const Vec3& value) const;
    void set(Location loc, const Vec4& value) const;
    void set(Location loc, const Mat22& value) const;
    void set(Location loc, const Mat33& value) const;
    void set(Location loc, const Mat44& value) const;

    template<class T>
    void set(const char* name, const T& value) const { set(location(name), value); }

private:
    bool check_link_errors() const;

    RenderHandle m_id = 0;
};

using ShaderProgramPtr = std::shared_ptr<ShaderProgram>;

} // slope::app