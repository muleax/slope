#include "app/system/app.hpp"
#include "app/system/utils.hpp"
#include "slope/debug/log.hpp"
#include "slope/debug/assert.hpp"
#include "glad/gl.h"
#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"
#include "imgui_backends/glfw.hpp"
#include "imgui_backends/opengl3.hpp"

namespace slope::app {

class AppCallbacks {
    static_assert(std::is_same_v<std::underlying_type_t<Key>, int>);
    static_assert(std::is_same_v<std::underlying_type_t<MouseButton>, int>);
    static_assert(std::is_same_v<std::underlying_type_t<KeyAction>, int>);
    static_assert(std::is_same_v<KeyMod::Raw, int>);

public:
    static void set_instance(App* app_instance) {
        SL_VERIFY(s_app_instance == nullptr || app_instance == nullptr);
        s_app_instance = app_instance;
    }

    static void on_key(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (s_app_instance)
            s_app_instance->on_key(static_cast<Key>(key), scancode, static_cast<KeyAction>(action), mods);
    }

    static void on_mouse_button(GLFWwindow* window, int button, int action, int mods) {
        if (s_app_instance && !ImGui::GetIO().WantCaptureMouse)
            s_app_instance->on_mouse_button(static_cast<MouseButton>(button), static_cast<KeyAction>(action), mods);
    }

    static void on_cursor_pos(GLFWwindow* window, double x_pos, double y_pos) {
        if (s_app_instance) {
            s_app_instance->move_cursor(x_pos, y_pos);
        }
    }

    static void on_cursor_enter(GLFWwindow* window, int entered) {
        if (s_app_instance)
            s_app_instance->on_cursor_enter(entered == GLFW_TRUE);
    }

    static void on_scroll(GLFWwindow* window, double x_offset, double y_offset) {
        if (s_app_instance)
            s_app_instance->on_scroll(x_offset, y_offset);
    }

    static void on_glfw_error(int error, const char* description) {
        log::error("GLFW error: {}", description);
    }

private:
    static App* s_app_instance;
};

App* AppCallbacks::s_app_instance = nullptr;

void App::init(const AppCfg& cfg) {
    SL_VERIFY(glfwInit());

    AppCallbacks::set_instance(this);
    glfwSetErrorCallback(AppCallbacks::on_glfw_error);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    m_window = glfwCreateWindow(cfg.window_width, cfg.window_height, cfg.title.c_str(), nullptr, nullptr);
    SL_VERIFY(m_window != nullptr);

    glfwSetKeyCallback(m_window, AppCallbacks::on_key);
    glfwSetMouseButtonCallback(m_window, AppCallbacks::on_mouse_button);
    glfwSetCursorPosCallback(m_window, AppCallbacks::on_cursor_pos);
    glfwSetCursorEnterCallback(m_window, AppCallbacks::on_cursor_enter);
    glfwSetScrollCallback(m_window, AppCallbacks::on_scroll);

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(0);

    gladLoadGL(glfwGetProcAddress);

    // setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init();

    ImGui::StyleColorsDark();

    //ImGui::LoadIniSettingsFromDisk("test.ini");

    m_start_time = glfwGetTime();

    glfwGetCursorPos(m_window, &m_x_cursor_pos, &m_y_cursor_pos);

    on_init();
    update_window_size();
}

App::~App() {
    //ImGui::SaveIniSettingsToDisk("test.ini");

    AppCallbacks::set_instance(nullptr);
}

void App::run() {
    double prev_time = get_time() - 1e-4;

    while (!glfwWindowShouldClose(m_window)) {
        update_window_size();

        glViewport(0, 0, m_win_width, m_win_height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        double curr_time = get_time();
        auto dt = static_cast<float>(curr_time - prev_time);
        prev_time = curr_time;

        update(dt);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
        glfwPollEvents();
    }

    ImGui_ImplGlfw_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void App::stop() {
    glfwSetWindowShouldClose(m_window, true);
}

void App::get_window_size(int& width, int& height) const {
    width = m_win_width;
    height = m_win_height;
}

void App::get_cursor_pos(double& x, double& y) const {
    x = m_x_cursor_pos;
    y = m_y_cursor_pos;
}

void App::set_cursor_pos(double x, double y) {
    m_x_cursor_pos = x;
    m_y_cursor_pos = y;
    glfwSetCursorPos(m_window, x, y);
}

void App::move_cursor(double x, double y) {
    double dx = x - m_x_cursor_pos;
    double dy = y - m_y_cursor_pos;
    m_x_cursor_pos = x;
    m_y_cursor_pos = y;

    on_cursor_move(dx, dy);
}

void App::update_window_size() {
    int width;
    int height;
    glfwGetFramebufferSize(m_window, &width, &height);

    if (width != m_win_width || height != m_win_height) {
        m_win_width = width;
        m_win_height = height;
        on_window_resize(width, height);
    }
}

void App::set_background_color(const vec3& rgb) {
    glClearColor(rgb.x, rgb.y, rgb.z, 0.f);
}

} // slope::app