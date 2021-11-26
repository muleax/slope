#include "app/scene/ui_overlay_system.hpp"
#include "app/scene/physics_system.hpp"
#include "app/render/render_system.hpp"
#include "app/system/utils.hpp"
#include "imgui/imgui.h"

namespace slope::app {

void UIOverlaySystem::update(float dt) {
    auto* ps = w().modify_singleton<PhysicsSingleton>();
    auto* rs = w().modify_singleton<RenderSingleton>();

    if (ps) {
        auto& world_stats = ps->dynamics_world.stats();
        auto& world_config = ps->dynamics_world.config();

        ImGui::Begin("Simulation");
        ImGui::Text("Simulation Time: %.1f s", world_stats.simulation_time);
        ImGui::Text("CPU Frame Time: %.1f ms", ps->cpu_time * 1000);

        ImGui::Text("Static Actors: %d", world_stats.static_actor_count);
        ImGui::Text("Dynamic Actors: %d", world_stats.dynamic_actor_count);
        ImGui::Text("Collisions: %d", world_stats.collision_count);
        ImGui::Text("Contacts: %d", world_stats.contact_count);

        ImGui::Separator();
        ImGui::Checkbox("Pause", &ps->pause);
        ImGui::Checkbox("Real Time Sync", &ps->real_time_sync);
        ImGui::Checkbox("Initial Randomization", &world_config.randomize_order);

        bool use_gjk = ps->dynamics_world.narrowphase().preferred_backend() == Narrowphase::Backend::GJK_EPA;
        if (ImGui::RadioButton("SAT", !use_gjk))
            ps->dynamics_world.narrowphase().set_preferred_backend(Narrowphase::Backend::SAT);
        ImGui::SameLine();
        if (ImGui::RadioButton("GJK+EPA", use_gjk))
            ps->dynamics_world.narrowphase().set_preferred_backend(Narrowphase::Backend::GJK_EPA);

        ImGui::DragFloat("Time Factor", &ps->time_factor, 0.01f, 0.001f, 100.f);
        float time_step_ms = ps->time_step * 1000.f;
        ImGui::DragFloat("Time Step", &time_step_ms, 0.1f, 0.1f, 1000.f);
        ps->time_step = time_step_ms * 0.001f;

        ImGui::DragInt("Iterations", &world_config.iteration_count, 0.5f, 1, 300);
        ImGui::DragFloat("WS Normal", &world_config.warmstarting_normal, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("WS Friction", &world_config.warmstarting_friction, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("SOR", &world_config.sor, 0.001f, 0.f, 1.f);
        ImGui::InputFloat3("Gravity", world_config.gravity.data);

        ImGui::End();
    }

    if (rs && ps) {
        auto& world_config = ps->dynamics_world.config();

        ImGui::Begin("Render");

        ImGui::Checkbox("Wireframe", &rs->wireframe);
        ImGui::Checkbox("Contact Normals", &world_config.draw_contact_normals);
        ImGui::Checkbox("Contact Friction", &world_config.draw_contact_friction);

        ImGui::End();
    }
}

} // slope::app
