#include "app/render/shader_program.hpp"
#include "slope/debug/assert.hpp"
#include "slope/debug/log.hpp"
#include "slope/containers/array.hpp"
#include "glad/gl.h"
#include <optional>

namespace slope::app {

static_assert(std::is_same_v<RenderHandle, GLuint>);
static_assert(std::is_same_v<ShaderProgram::Location, GLint>);

class Shader {
public:
    Shader(GLenum shader_type, const char* source);
    ~Shader();

    bool    ready() const { return m_id != 0; }
    GLuint  id() const { return m_id; }

private:
    bool        check_compile_errors() const;
    const char* type_name() const;

    GLenum  m_shader_type = 0;
    GLuint  m_id = 0;
};

Shader::Shader(GLenum shader_type, const char* source) : m_shader_type(shader_type) {
    m_id = glCreateShader(shader_type);
    glShaderSource(m_id, 1, &source, nullptr);
    glCompileShader(m_id);
    if (!check_compile_errors()) {
        glDeleteShader(m_id);
        m_id = 0;
    }
}

Shader::~Shader() {
    if (m_id != 0) {
        glDeleteShader(m_id);
    }
}

const char* Shader::type_name() const {
    if (m_shader_type == GL_VERTEX_SHADER) {
        return "vertex";
    } else if (m_shader_type == GL_FRAGMENT_SHADER) {
        return "fragment";
    } else {
        return "unknown";
    }
}

bool Shader::check_compile_errors() const {
    GLint success;
    Array<GLchar, 1024> msg;
    glGetShaderiv(m_id, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(m_id, 1024, nullptr, msg.data());
        log::error("Failed to compile {} shader: {}", type_name(), msg.data());
    }

    return success;
}

ShaderProgram::ShaderProgram(const char* vertex_shader_src, const char* fragment_shader_src)
{
    Shader vertex_shader(GL_VERTEX_SHADER, vertex_shader_src);
    SL_VERIFY(vertex_shader.ready());

    Shader fragment_shader(GL_FRAGMENT_SHADER, fragment_shader_src);
    SL_VERIFY(fragment_shader.ready());

    m_id = glCreateProgram();
    glAttachShader(m_id, vertex_shader.id());
    glAttachShader(m_id, fragment_shader.id());

    glLinkProgram(m_id);
    if (!check_link_errors()) {
        glDeleteProgram(m_id);
        m_id = 0;
    }
}

ShaderProgram::~ShaderProgram() {
    if (m_id != 0) {
        glDeleteProgram(m_id);
    }
}

bool ShaderProgram::check_link_errors() const {
    GLint success;
    Array<GLchar, 1024> msg;
    glGetProgramiv(m_id, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(m_id, msg.size(), nullptr, msg.data());
        log::error("Failed to link shader program: {}", msg.data());
    }

    return success;
}

void ShaderProgram::use() const {
    SL_ASSERT(ready());
    glUseProgram(m_id);
}

ShaderProgram::Location ShaderProgram::location(const char* name) const {
    return glGetUniformLocation(m_id, name);
}

ShaderProgram::Location ShaderProgram::attribute_location(const char* name) const {
    return glGetAttribLocation(m_id, name);
}

void ShaderProgram::set(Location loc, bool value) const
{
    glUniform1i(loc, static_cast<int>(value));
}

void ShaderProgram::set(Location loc, int value) const
{
    glUniform1i(loc, value);
}

void ShaderProgram::set(Location loc, float value) const
{
    glUniform1f(loc, value);
}

void ShaderProgram::set(Location loc, const Vec2& value) const
{
    glUniform2fv(loc, 1, value.data);
}

void ShaderProgram::set(Location loc, const Vec3& value) const
{
    glUniform3fv(loc, 1, value.data);
}

void ShaderProgram::set(Location loc, const Vec4& value) const
{
    glUniform4fv(loc, 1, value.data);
}

void ShaderProgram::set(Location loc, const Mat22& mat) const
{
    glUniformMatrix2fv(loc, 1, GL_FALSE, mat.data);
}

void ShaderProgram::set(Location loc, const Mat33& mat) const
{
    glUniformMatrix3fv(loc, 1, GL_FALSE, mat.data);
}

void ShaderProgram::set(Location loc, const Mat44& mat) const
{
    glUniformMatrix4fv(loc, 1, GL_FALSE, mat.data);
}

} // slope::app