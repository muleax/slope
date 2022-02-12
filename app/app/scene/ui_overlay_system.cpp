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

    auto& stats = ps->dynamics_world.stats();
    auto& np_stats = stats.np_stats;

    auto& gjk_stats = np_stats.gjk_stats;
    auto& epa_stats = np_stats.epa_stats;
    auto& sat_stats = np_stats.sat_stats;

    auto treeNodeFlags = ImGuiTreeNodeFlags_DefaultOpen;

    ImGui::SetNextWindowPos(ImVec2(340, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(230, 550), ImGuiCond_FirstUseEver);
    ImGui::Begin("Stats");

    if (ImGui::CollapsingHeader("General", treeNodeFlags)) {
        ImGui::Text("Simulation Time: %.1f s", stats.simulation_time);
        ImGui::Text("CPU Frame Time: %.1f ms", ps->cpu_time.instantaneous() * 1000);
        ImGui::Text("Kinematic Actors: %d", stats.kinematic_actor_count);
        ImGui::Text("Dynamic Actors: %d", stats.dynamic_actor_count);
        ImGui::Text("Joints: %d", stats.joint_count);
    }

    if (ImGui::CollapsingHeader("Narrowphase", treeNodeFlags)) {
        ImGui::Text("Narrowphase Tests: %d", np_stats.test_count);
        ImGui::Text("Collisions: %d", np_stats.collision_count);
        ImGui::Text("Contact points: %d", np_stats.contact_count);

        ImGui::Text("GJK Tests: %llu", gjk_stats.cum_test_count);
        ImGui::Text("GJK Total Fails: %llu", gjk_stats.total_fail_count);
        ImGui::Text("GJK Max Iterations: %d", gjk_stats.max_iteration_count);
        ImGui::Text("GJK Avg Iterations: %.2f", float(gjk_stats.cum_iterations_count) / float(gjk_stats.cum_test_count));

        ImGui::Text("EPA Tests: %llu", epa_stats.cum_test_count);
        ImGui::Text("EPA Total Fails: %llu", epa_stats.total_fail_count);
        ImGui::Text("EPA Max Iterations: %d", epa_stats.max_iteration_count);
        ImGui::Text("EPA Avg Iterations: %.2f", float(epa_stats.cum_iterations_count) / float(epa_stats.cum_test_count));

        ImGui::Text("SAT Tests: %llu", sat_stats.cum_test_count);
        ImGui::Text("SAT Avg Projections: %.2f", float(sat_stats.cum_projection_count) / float(sat_stats.cum_test_count));
    }

    if (ImGui::CollapsingHeader("Islands", treeNodeFlags)) {
        ImGui::Text("Island Count: %d", stats.island_count);
        for (int i = 0; i < stats.larges_islands.size(); i++) {
            ImGui::Text("%d-Largest: %.3f", i + 1, stats.larges_islands[i] / float(stats.dynamic_actor_count));
        }
    }

    if (ImGui::CollapsingHeader("Solver", treeNodeFlags)) {
        ImGui::Text("Constraints: %d", stats.constraint_count);
    }

    ImGui::End();

    DynamicsWorldConfig world_config = ps->dynamics_world.config();
    auto& solver_config = world_config.solver_config;
    auto& np_config = world_config.np_config;
    auto& gjk_config = np_config.gjk_config;
    auto& epa_config = np_config.epa_config;

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 980), ImGuiCond_FirstUseEver);
    ImGui::Begin("Config");

    if (ImGui::CollapsingHeader("General", treeNodeFlags)) {
        ImGui::Checkbox("Pause", &ps->pause);
        ImGui::Checkbox("Enable Constraint Resolving", &world_config.enable_constraint_resolving);
        ImGui::Checkbox("Enable Integration", &world_config.enable_integration);
        ImGui::Checkbox("Real Time Sync", &ps->real_time_sync);

        ImGui::DragFloat("Time Factor", &ps->time_factor, 0.01f, 0.001f, 100.f);
        float time_step_ms = world_config.solver_config.time_interval * 1000.f;

        ImGui::DragFloat("Time Step", &time_step_ms, 0.1f, 0.1f, 1000.f);
        world_config.solver_config.time_interval = time_step_ms * 0.001f;

        ImGui::Checkbox("Gyroscopic Torque", &world_config.enable_gyroscopic_torque);
        ImGui::Checkbox("Enable Gravity", &world_config.enable_gravity);
        ImGui::InputFloat3("Gravity", world_config.gravity.data);
    }

    if (ImGui::CollapsingHeader("Narrowphase", treeNodeFlags)) {
        ImGui::Text("Policy:");
        ImGui::SameLine();
        bool use_mixed = (world_config.np_backend_hint == NpBackendHint::Mixed);
        if (ImGui::RadioButton("Mixed", use_mixed))
            world_config.np_backend_hint = NpBackendHint::Mixed;
        ImGui::SameLine();
        bool use_gjk = (world_config.np_backend_hint == NpBackendHint::GJK_EPA);
        if (ImGui::RadioButton("GJK/EPA", use_gjk))
            world_config.np_backend_hint = NpBackendHint::GJK_EPA;
        ImGui::SameLine();
        bool use_sat = (world_config.np_backend_hint == NpBackendHint::SAT);
        if (ImGui::RadioButton("SAT", use_sat))
            world_config.np_backend_hint = NpBackendHint::SAT;

        ImGui::DragScalar("GJK Max Iters", ImGuiDataType_U32, &gjk_config.max_iteration_count);
        ImGui::DragScalar("EPA Max Iters", ImGuiDataType_U32, &epa_config.max_iteration_count);
        ImGui::DragFloat("EPA Support Bloat", &epa_config.support_bloat);
        ImGui::DragFloat("EPA Early Threshold", &epa_config.early_threshold, 1e-7f, -1.f, 1.f, "%.8f");
        ImGui::DragFloat("EPA Final Threshold", &epa_config.final_threshold, 1e-4f, -1.f, 1.f, "%.8f");
    }

    if (ImGui::CollapsingHeader("Solver", treeNodeFlags)) {
        ImGui::Checkbox("Initial Randomization", &world_config.randomize_order);
        ImGui::Checkbox("Velocity Dependent Friction", &world_config.enable_velocity_dependent_friction);
        ImGui::Checkbox("Cone Friction", &world_config.enable_cone_friction);

        ImGui::DragInt("Iterations", &solver_config.iteration_count, 0.5f, 1, 300);
        ImGui::DragFloat("SOR", &solver_config.sor, 0.001f, 0.f, 1.f);
        ImGui::DragFloat("Normal WS", &world_config.normal_warmstarting, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("Friction WS", &world_config.friction_warmstarting, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("Contact ERP", &world_config.contact_erp, 0.001f, 0.f, 1.f);
        ImGui::DragFloat("Contact CFM", &world_config.contact_cfm, 0.001f, 0.f, 1.f);
        ImGui::DragFloat("Contact Penetration", &world_config.contact_penetration, 0.001f, 0.f, 1.f);
        ImGui::DragFloat("Joint WS", &world_config.joint_warmstarting, 0.002f, 0.f, 1.f);
        ImGui::DragFloat("Joint ERP", &world_config.joint_erp, 0.002f, 0.f, 1.f);
    }

    if (ImGui::CollapsingHeader("Performance", treeNodeFlags)) {
        ImGui::DragInt("Concurrency", &ps->concurrency, 1.f, 1, 8);
        ImGui::Checkbox("Use SIMD Solver", &world_config.solver_config.use_simd);
    }

    auto* rs = w().modify_singleton<RenderSingleton>();
    if (rs) {
        if (ImGui::CollapsingHeader("Visualization", treeNodeFlags)) {
            ImGui::Checkbox("Wireframe", &rs->wireframe);
            ImGui::Checkbox("Contact Normals 1", &world_config.draw_contact_normals1);
            ImGui::Checkbox("Contact Friction 1", &world_config.draw_contact_friction1);
            ImGui::Checkbox("Contact Normals 2", &world_config.draw_contact_normals2);
            ImGui::Checkbox("Contact Friction 2", &world_config.draw_contact_friction2);
            ImGui::Checkbox("Delay Integration", &world_config.delay_integration);
        }
    }

    ImGui::End();

    ps->dynamics_world.update_config(world_config);

    // ImGui::ShowDemoWindow();
}

} // slope::app
