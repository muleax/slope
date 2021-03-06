#pragma once
#include "app/system/input.hpp"
#include "slope/core/string.hpp"
#include "slope/math/vector3.hpp"

struct GLFWwindow;

namespace slope::app {

struct AppCfg {
    int window_width;
    int window_height;
    String title;
};

class App {
public:
    virtual ~App();

    virtual void update(float dt) = 0;

    virtual void on_init() {};

    virtual void on_key(Key key, int scancode, KeyAction action, KeyMod::Raw mods) {}
    virtual void on_mouse_button(MouseButton button, KeyAction action, KeyMod::Raw mods) {}
    virtual void on_cursor_move(double x_delta, double y_delta) {}
    virtual void on_cursor_enter(bool entered) {}
    virtual void on_scroll(double x_offset, double y_offset) {}
    virtual void on_window_resize(int width, int height) {}

    void    get_window_size(int& width, int& height) const;
    void    get_cursor_pos(double& x, double& y) const;
    void    set_cursor_pos(double x, double y);
    void    set_background_color(const vec3& rgb);

    void    stop();

private:
    void init(const AppCfg& cfg);
    void run();

    void update_window_size();
    void move_cursor(double x, double y);

    double m_x_cursor_pos = 0.0;
    double m_y_cursor_pos = 0.0;

    int m_win_width = 0;
    int m_win_height = 0;
    double m_start_time = 0.f;
    GLFWwindow* m_window = nullptr;

    friend class AppManager;
    friend class AppCallbacks;
};

class AppManager {
public:
    template<class TApp>
    static void run(const AppCfg& cfg) {
        static_assert(std::is_base_of_v<App, TApp>);
        TApp app;
        app.App::init(cfg);
        app.App::run();
    }
};

} // slope