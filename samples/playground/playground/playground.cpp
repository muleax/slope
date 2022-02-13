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
        vec3 eye = {0.f, 8.f, 10.f};
        m_cam_entity = m_world->create_entity();
        m_static_entities.push_back(m_cam_entity);
        m_world->create<TransformComponent>(m_cam_entity)->transform = mat44::translate(eye);
        m_world->create<CameraControllerComponent>(m_cam_entity);
        m_world->create<CameraComponent>(m_cam_entity);

        m_spawner = std::make_unique<BodySpawner>(m_world.get());

        // create demos
        register_demos<
            TriangleStackDemo,
            StackDemo,
            Stress1KDemo,
            Stress6KDemo,
            Stress10KDemo,
            SphericalJointDemo,
            TennisRacketDemo,
            ContactGenerationDemo>();

        m_selected_demo = m_demos[0].get();
        reset_demo();
    }

    void update(float dt) override
    {
        show_ui();
        m_selected_demo->update(dt);
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

private:
    template<class... Ts>
    void register_demos()
    {
        (m_demos.push_back(std::make_unique<Ts>(m_spawner.get())), ...);
    }

    void show_ui()
    {
        auto treeNodeFlags = ImGuiTreeNodeFlags_DefaultOpen;

        ImGui::SetNextWindowPos(ImVec2(340, 580), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(230, 400), ImGuiCond_FirstUseEver);
        ImGui::Begin("Playground");

        if (ImGui::CollapsingHeader("Demos", treeNodeFlags)) {
            if (ImGui::BeginCombo("##demos_combo", m_selected_demo->name())) {
                for (auto& demo: m_demos) {
                    bool is_selected = (m_selected_demo == demo.get());
                    if (ImGui::Selectable(demo->name(), is_selected))
                        m_selected_demo = demo.get();

                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            if (ImGui::Button(" RESET ")) {
                reset_demo();
            }
            ImGui::Checkbox("Default Config", &m_apply_default_demo_config);
        }

        if (ImGui::CollapsingHeader("Controls", treeNodeFlags)) {
            ImGui::Text("WASD         Camera move");
            ImGui::Text("LMB + Mouse  Camera rotation");
            ImGui::Text("Space        Pause simulation");
            ImGui::Text("F            Fire box");
            ImGui::Text("G            Fire sphere");
            ImGui::Text("H            Fire capsule");
        }

        if (ImGui::CollapsingHeader("Fire Object", treeNodeFlags)) {
            ImGui::DragFloat("Speed", &m_fire_object_speed, 0.1f, 0.f, 1000.f);
            ImGui::DragFloat("Mass", &m_fire_object_mass, 0.1f, 0.1f, 10000.f);
            ImGui::DragFloat("Size", &m_fire_object_size, 0.1f, 0.1f, 100.f);
        }

        if (ImGui::CollapsingHeader("Camera", treeNodeFlags)) {
            auto* ctl = m_world->modify<CameraControllerComponent>(m_cam_entity);
            ImGui::DragFloat("Cam Speed", &ctl->velocity, 0.2f, 0.f, 100.f);
        }

        ImGui::End();
    }

    void reset_demo()
    {
        if (m_current_demo) {
            m_current_demo->fini();
        }

        m_world->visit_entities([this](Entity e) {
            if (std::find(m_static_entities.begin(), m_static_entities.end(), e) == m_static_entities.end())
                m_world->destroy_entity(e);
        });

        auto* physics_single = m_world->modify_singleton<PhysicsSingleton>();
        // TODO: use fini view instead
        physics_single->dynamics_world.clear();
        physics_single->dynamics_world.clear_stats();

        m_selected_demo->init();

        if (m_apply_default_demo_config)
            m_selected_demo->apply_default_config();

        m_current_demo = m_selected_demo;
    }

    void fire_box()
    {
        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -m_fire_object_speed});
        m_spawner->spawn_box(cam_tr->transform, vel, m_fire_object_mass, vec3{m_fire_object_size});
    }

    void fire_sphere()
    {
        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -m_fire_object_speed});
        m_spawner->spawn_sphere(cam_tr->transform, vel, m_fire_object_mass, m_fire_object_size);
    }

    void fire_capsule()
    {
        auto* cam_tr = m_world->get<TransformComponent>(m_cam_entity);
        auto vel = cam_tr->transform.apply_normal({0.f, 0.f, -m_fire_object_speed});
        vec3 ang_vel = cam_tr->transform.apply_normal({0.f, 0.f, 0.f});
        m_spawner->spawn_capsule(cam_tr->transform, vel, ang_vel,
                                 m_fire_object_mass, m_fire_object_size * 0.5f, m_fire_object_size);
    }

    bool m_cam_move_mode = false;
    Entity m_cam_entity;

    Vector<Entity> m_static_entities;

    std::unique_ptr<BodySpawner> m_spawner;
    Vector<std::unique_ptr<Demo>> m_demos;
    Demo* m_current_demo = nullptr;
    Demo* m_selected_demo = nullptr;
    bool m_apply_default_demo_config = true;

    float m_fire_object_speed = 15.f;
    float m_fire_object_mass = 4.f;
    float m_fire_object_size = 1.5f;
    ShapeKind m_fire_object_kind = ShapeKind::Box;

    std::unique_ptr<World> m_world;
};

int main() {
    AppManager::run<PlaygroundApp>({1800, 1000, "Playground"});
    return 0;
}
