#pragma once
#include "app/render/config.hpp"
#include "slope/math/math.hpp"
#include <memory>

namespace slope::app {

class Shader
{
public:
    using Location = int32_t;

    Shader(const char* vertex_shader_src, const char* fragment_shader_src);
    virtual ~Shader();

    bool ready() const { return m_id != 0; }
    void use();

    Location attribute_location(const char* name) const;
    Location location(const char* name) const;

    void set(Location loc, bool value) const;
    void set(Location loc, int value) const;
    void set(Location loc, float value) const;
    void set(Location loc, const vec2& value) const;
    void set(Location loc, const vec3& value) const;
    void set(Location loc, const vec4& value) const;
    void set(Location loc, const mat22& value) const;
    void set(Location loc, const mat33& value) const;
    void set(Location loc, const mat44& value) const;

    template<class T>
    void set(const char* name, const T& value) const { set(location(name), value); }

protected:
    virtual void cache_attribute_locations() {}

private:
    bool check_link_errors() const;

    RenderHandle m_id = 0;
    bool m_first_use = true;
};

} // slope::app
