#include "playground/helpers.hpp"
#include "playground/demos.hpp"
#include "app/system/app.hpp"
#include "app/render/render_system.hpp"
#include "app/render/camera.hpp"
#include "app/render/debug_drawer_impl.hpp"
#include "app/scene/transform.hpp"
#include "app/scene/physics_system.hpp"
#include "app/scene/ui_overlay_system.hpp"
#include "app/ecs/world.hpp"
#include "slope/debug/log.hpp"
#include "imgui/imgui.h"
#include <memory>

using namespace slope;
using namespace slope::app;

class PlaygroundApp : public App {
public:
    ~PlaygroundApp() override = default;

    template<class... Ts>
    void register_demos()
    {
        (m_demos.push_back(std::make_unique<Ts>(m_spawner.get())), ...);
    }

    void on_init() override
    {
        m_world = std::make_unique<World>();
        m_world->add_system<PhysicsSystem>();
        m_world->add_system<CameraSystem>();
        m_world->add_system<RenderSystem>();
        m_world->add_system<UIOverlaySystem>();
        m_world->create_singleton<RenderSingleton>();
        m_world->create_singleton<PhysicsSingleton>();

        set_background_color({0.2f, 0.2, 0.2f});

        // create debug drawer
        auto dde = m_world->create_entity();
        m_static_entities.push_back(dde);
        auto* debug_draw = m_world->create<DebugDrawComponent>(dde);
        debug_draw->drawer = std::make_shared<DebugDrawerImpl>();
        m_world->modify_singleton<PhysicsSingleton>()->dynamics_world.set_debug_drawer(debug_draw->drawer);

        // create light source
        auto le = m_world->create_entity();
        m_static_entities.push_back(le);
        m_world->create<LightSourceComponent>(le);
        m_world->create<TransformComponent>(le)->transform = mat44::translate({50.f, 100.f, 70.f});

        // create camera
        vec3 eye = {4.f, 5.f, 10.f};
        m_cam_entity = m_world->create_entity();
        m_static_entities.push_back(m_cam_entity);
        m_world->create<TransformComponent>(m_cam_entity)->transform = mat44::translate(eye);
        m_world->create<CameraControllerComponent>(m_cam_entity);
        m_world->create<CameraComponent>(m_cam_entity);

        m_spawner = std::make_unique<BodySpawner>(m_world.get());

        // create demos
        register_demos<TriangleStackDemo, StackDemo, StressTestDemo, CollisionDemo, TennisRacketDemo, SphericalJointDemo>();

        m_current_demo = m_demos[2].get();
        reset_demo();
    }

    void update(float dt) override
    {
        show_demos_ui();
        m_current_demo->update(dt);
        m_world->update(dt);
    }

    void on_window_resize(int width, int height) override
    {
        auto& cam = m_world->modify<CameraComponent>(m_cam_entity)->camera;
        cam.set_aspect_ratio(static_cast<float>(width) / height);
    }

    void on_key(Key key, int scancode, KeyAction action, int mods) override
    {
        bool is_pressed = action == KeyAction::Press || action == KeyAction::Repeat;

        auto* cam_ctl = m_world->modify<CameraControllerComponent>(m_cam_entity);

        switch (key) {
        case Key::A:
            cam_ctl->move_left = is_pressed;
            break;
        case Key::D:
            cam_ctl->move_right = is_pressed;
            break;
        case Key::W:
            cam_ctl->move_fwd = is_pressed;
            break;
        case Key::S:
            cam_ctl->move_bkwd = is_pressed;
            break;

        case Key::LeftShift: {
            auto* cam = m_world->modify<CameraControllerComponent>(m_cam_entity);
            cam->velocity = is_pressed ? 1.f : 8.f;
            break;
        }

        case Key::F:
            if (is_pressed)
                fire_box();
            break;

        case Key::G:
            if (is_pressed)
                fire_sphere();
            break;

        case Key::H:
            if (is_pressed)
                fire_capsule();
            break;

        case Key::Space:
            if (action == KeyAction::Press) {
                auto* phyics_single = m_world->modify_singleton<PhysicsSingleton>();
                phyics_single->pause = !phyics_single->pause;
            }
            break;
        default:
            break;
        }
    }

    void on_cursor_move(double x_delta, double y_delta) override
    {
        if (m_cam_move_mode) {
            auto* cam_ctl = m_world->modify<CameraControllerComponent>(m_cam_entity);
            cam_ctl->rotate(x_delta, -y_delta);
        }
    }

    void on_mouse_button(MouseButton button, KeyAction action, KeyMod::Raw mods) override
    {
        if (button == MouseButton::B_1) {
            m_cam_move_mode = (action == KeyAction::Press);
        }
    }

    void show_demos_ui()
    {
        ImGui::Begin("Demos");

        if (ImGui::BeginCombo("##demos_combo", m_current_demo->name())) {
            for (auto& demo: m_demos) {
                bool is_selected = (m_current_demo == demo.get());
                if (ImGui::Selectable(demo->name(), is_selected))
                    m_current_demo = demo.get();

                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button("Reset")) {
            reset_demo();
        }

        ImGui::End();
    }

    void reset_demo()
    {
        m_world->visit_entities([this](Entity e) {
            if (std::find(m_static_entities.begin(), m_static_entities.end(), e) == m_static_entities.end())
                m_world->destroy_entity(e);
        });

        auto* physics_single = m_world->modify_singleton<PhysicsSingleton>();
        physics_single->dynamics_world.update_config({});
        // TODO: use fini view instead
        physics_single->dynamics_world.clear();

        m_current_demo->init();
    }

    void fire_box()
    {
        static constexpr float BOX_SPEED = 15.f;

        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -BOX_SPEED});
        m_spawner->spawn_box(cam_tr->transform, vel, 5.f, vec3{1.5f});
    }

    void fire_sphere()
    {
        static constexpr float SPHERE_SPEED = 15.f;

        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -SPHERE_SPEED});
        m_spawner->spawn_sphere(cam_tr->transform, vel, 1.f, 1.f);
    }

    void fire_capsule()
    {
        static constexpr float CAPSULE_SPEED = 20.f;

        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -CAPSULE_SPEED});
        vec3 ang_vel = cam_tr->transform.apply_normal({0.f, 8.f, 0.f});
        m_spawner->spawn_capsule(cam_tr->transform, vel, ang_vel, 1.f, 1.f, 2.f);
    }

    bool m_cam_move_mode = false;
    Entity m_cam_entity;

    Vector<Entity> m_static_entities;

    std::unique_ptr<BodySpawner> m_spawner;
    Vector<std::unique_ptr<Demo>> m_demos;
    Demo* m_current_demo = nullptr;

    std::unique_ptr<World> m_world;
};

int main() {
    AppManager::run<PlaygroundApp>({1600, 1200, "Playground"});
    return 0;
}
