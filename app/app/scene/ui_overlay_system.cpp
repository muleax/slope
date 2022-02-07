#include "app/scene/ui_overlay_system.hpp"
#include "app/scene/physics_system.hpp"
#include "app/render/render_system.hpp"
#include "app/system/utils.hpp"
#include "imgui/imgui.h"

namespace slope::app {

void UIOverlaySystem::update(float dt)
{
    auto* ps = w().modify_singleton<PhysicsSingleton>();
    if (!ps)
        return;

    auto& np = ps->dynamics_world.narrowphase();

    auto& world_stats = ps->dynamics_world.stats();
    auto& gjk_stats = np.gjk_solver().stats();
    auto& epa_stats = np.epa_solver().stats();
    auto& sat_stats = np.sat_solver().stats();

    auto& world_config = ps->dynamics_world.config();
    auto& solver_config = world_config.solver_config;
    auto& gjk_config = np.gjk_solver().config();
    auto& epa_config = np.epa_solver().config();

    auto treeNodeFlags = ImGuiTreeNodeFlags_DefaultOpen;

    ImGui::Begin("Stats");

    // ImGui::SetNextItemOpen(true);
    if (ImGui::CollapsingHeader("General", treeNodeFlags)) {
        ImGui::Text("Simulation Time: %.1f s", world_stats.simulation_time);
        ImGui::Text("CPU Frame Time: %.1f ms", ps->cpu_time.instantaneous() * 1000);
        ImGui::Text("Static Actors: %d", world_stats.actor_count[(int)ActorKind::Static]);
        ImGui::Text("Dynamic Actors: %d", world_stats.actor_count[(int)ActorKind::Dynamic]);
    }

    if (ImGui::CollapsingHeader("Narrowphase", treeNodeFlags)) {
        ImGui::Text("Narrowphase Tests: %d", world_stats.np_test_count);
        ImGui::Text("Collisions: %d", world_stats.collision_count);
        ImGui::Text("Contacts: %d", world_stats.contact_count);

        ImGui::Text("GJK Tests: %llu", gjk_stats.cum_test_count);
        ImGui::Text("GJK Total Fails: %llu", gjk_stats.total_fail_count);
        ImGui::Text("GJK Max Iterations: %d", gjk_stats.max_iteration_count);
        ImGui::Text("GJK Avg Iterations: %f", float(gjk_stats.cum_iterations_count) / float(gjk_stats.cum_test_count));

        ImGui::Text("EPA Tests: %llu", epa_stats.cum_test_count);
        ImGui::Text("EPA Total Fails: %llu", epa_stats.total_fail_count);
        ImGui::Text("EPA Max Iterations: %d", epa_stats.max_iteration_count);
        ImGui::Text("EPA Avg Iterations: %f", float(epa_stats.cum_iterations_count) / float(epa_stats.cum_test_count));

        ImGui::Text("SAT Tests: %llu", sat_stats.cum_test_count);
        ImGui::Text("SAT Avg Projections: %f", float(sat_stats.cum_projection_count) / float(sat_stats.cum_test_count));
    }

    if (ImGui::CollapsingHeader("Constraints", treeNodeFlags)) {
        ImGui::Text("Max solver error: %f", world_stats.max_constraint_solver_error.smoothed());
        ImGui::Text("Avg solver error: %f", world_stats.avg_constraint_solver_error.smoothed());
    }

    ImGui::End();

    ImGui::Begin("Controls");

    if (ImGui::CollapsingHeader("Simulation", treeNodeFlags)) {
        ImGui::Checkbox("Pause", &ps->pause);
        ImGui::Checkbox("Real Time Sync", &ps->real_time_sync);
        ImGui::Checkbox("Enable Gravity", &world_config.enable_gravity);
        ImGui::Checkbox("Enable Constraint Resolving", &world_config.enable_constraint_resolving);
        ImGui::Checkbox("Enable Integration", &world_config.enable_integration);
        ImGui::Checkbox("Initial Randomization", &world_config.randomize_order);
        ImGui::Checkbox("Velocity Dependent Friction", &world_config.enable_velocity_dependent_friction);
        ImGui::Checkbox("Cone Friction", &world_config.enable_cone_friction);
        ImGui::Checkbox("Gyroscopic Torque", &world_config.enable_gyroscopic_torque);
        ImGui::Checkbox("Use SIMD solver", &world_config.solver_config.use_simd);

        bool use_mixed = (world_config.np_backend_hint == DynamicsWorld::NpBackendHint::Mixed);
        if (ImGui::RadioButton("Mixed", use_mixed))
            world_config.np_backend_hint = DynamicsWorld::NpBackendHint::Mixed;
        ImGui::SameLine();
        bool use_gjk = (world_config.np_backend_hint == DynamicsWorld::NpBackendHint::GJK_EPA);
        if (ImGui::RadioButton("GJK/EPA", use_gjk))
            world_config.np_backend_hint = DynamicsWorld::NpBackendHint::GJK_EPA;
        ImGui::SameLine();
        bool use_sat = (world_config.np_backend_hint == DynamicsWorld::NpBackendHint::SAT);
        if (ImGui::RadioButton("SAT", use_sat))
            world_config.np_backend_hint = DynamicsWorld::NpBackendHint::SAT;

        ImGui::DragInt("Concurrency", &ps->concurrency, 1.f, 1, 8);
        ImGui::DragScalar("GJK Max Iters", ImGuiDataType_U32, &gjk_config.max_iteration_count);
        ImGui::DragScalar("EPA Max Iters", ImGuiDataType_U32, &epa_config.max_iteration_count);
        ImGui::DragFloat("EPA Support Bloat", &epa_config.support_bloat);
        ImGui::DragFloat("EPA Early Threshold", &epa_config.early_threshold, 1e-7f, -1.f, 1.f, "%.8f");
        ImGui::DragFloat("EPA Final Threshold", &epa_config.final_threshold, 1e-4f, -1.f, 1.f, "%.8f");

        ImGui::DragFloat("Time Factor", &ps->time_factor, 0.01f, 0.001f, 100.f);
        float time_step_ms = ps->time_step * 1000.f;
        ImGui::DragFloat("Time Step", &time_step_ms, 0.1f, 0.1f, 1000.f);
        ps->time_step = time_step_ms * 0.001f;

        ImGui::DragInt("Iterations", &solver_config.iteration_count, 0.5f, 1, 300);
        ImGui::DragFloat("WS Normal", &world_config.warmstarting_normal, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("WS Friction", &world_config.warmstarting_friction, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("WS Joint", &world_config.warmstarting_joint, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("SOR", &solver_config.sor, 0.001f, 0.f, 1.f);
        ImGui::InputFloat3("Gravity", world_config.gravity.data);
    }

    auto* rs = w().modify_singleton<RenderSingleton>();
    if (rs) {
        if (ImGui::CollapsingHeader("Render", treeNodeFlags)) {
            ImGui::Checkbox("Wireframe", &rs->wireframe);
            ImGui::Checkbox("Contact Normals 1", &world_config.draw_contact_normals1);
            ImGui::Checkbox("Contact Friction 1", &world_config.draw_contact_friction1);
            ImGui::Checkbox("Contact Normals 2", &world_config.draw_contact_normals2);
            ImGui::Checkbox("Contact Friction 2", &world_config.draw_contact_friction2);
            ImGui::Checkbox("Delay Integration", &world_config.delay_integration);
        }
    }

    ImGui::End();

    // ImGui::ShowDemoWindow();
}

} // slope::app
