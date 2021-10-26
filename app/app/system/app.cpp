#include "app/system/app.hpp"
#include "slope/math/vector2.hpp"
#include "slope/math/vector3.hpp"
#include "slope/debug/log.hpp"
#include "slope/debug/assert.hpp"
#include "app/render/shader_program.hpp"
#include "glad/gl.h"
#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"

namespace slope::app {

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

static void glfw_error_callback(int error, const char* description) {
    //fprintf(stderr, "GLFW error: %s\n", description);
}

App::App(const AppCfg& cfg) {
    glfwSetErrorCallback(glfw_error_callback);

    SL_VERIFY(glfwInit());

    start_time = glfwGetTime();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    m_window = glfwCreateWindow(cfg.window_width, cfg.window_height, cfg.title.c_str(), nullptr, nullptr);
    SL_VERIFY(m_window != nullptr);

    glfwSetKeyCallback(m_window, key_callback);

    glfwMakeContextCurrent(m_window);
    gladLoadGL(glfwGetProcAddress);
    glfwSwapInterval(0);

    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_RASTERIZER_DISCARD);
}

App::~App() {
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void App::run() {
    double prev_time = get_time() - 1e-4;


    while (!glfwWindowShouldClose(m_window)) {
        int width;
        int height;
        get_frame_buffer_size(width, height);
        glViewport(0, 0, width, height);
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        double curr_time = get_time();
        auto dt = static_cast<float>(curr_time - prev_time);
        prev_time = curr_time;

        update(dt);

        glfwSwapBuffers(m_window);
        //glFinish();
        //glFlush();
        glfwPollEvents();

        static int frames = 0;
        static float cum_dt = 0;
        frames++;
        cum_dt += dt;
        if (cum_dt >= 1.f) {
            auto fps = static_cast<float>(frames) / cum_dt;
            auto avg_dt = 1000 * cum_dt / frames;
            printf("%f ms\n", avg_dt);
            frames = 0;
            cum_dt = 0.f;
        }
    }
}

void App::get_frame_buffer_size(int& out_width, int& out_height) const {
    glfwGetFramebufferSize(m_window, &out_width, &out_height);
}

double App::get_time() const {
    return glfwGetTime() - start_time;
}

} // slope::app