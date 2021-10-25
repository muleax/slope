#pragma once
#include "slope/containers/string.hpp"

struct GLFWwindow;

namespace slope::app {

struct AppCfg {
    int window_width;
    int window_height;
    String title;
};

class App {
public:
    App(const AppCfg& cfg);
    ~App();

    void    run();
    void    get_frame_buffer_size(int& out_width, int& out_height) const;
    double  get_time() const;

    virtual void update(float dt) = 0;

private:
    GLFWwindow* m_window;
    double start_time = 0.f;
};

} // slope