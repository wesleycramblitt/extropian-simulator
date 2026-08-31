// ─────────────────────────────────────────────────────────────────────
// Headless test for the spatial-ui dashboard pipeline of the physics
// demos: DashboardFeedSystem (exd::sim) → scene_renderer resolution
// (Size → Layout → Mesh, with the headless MeshCreateFn injection) must
// materialize widget meshes for the dashboard chart + panel.
// ─────────────────────────────────────────────────────────────────────
#include <exd/sim/dashboard_feed_system.hpp>
#include <exd/sim/components/engine.hpp>
#include <exd/sim/components/solver_run.hpp>

#include <exd/scene_renderer/components/components.hpp>
#include <exd/scene_renderer/render_queue.hpp>
#include <exd/scene_renderer/systems/layout_system.hpp>
#include <exd/scene_renderer/systems/mesh_system.hpp>
#include <exd/scene_renderer/systems/size_system.hpp>
#include <exd/render/components/ui_renderable.hpp>

#include <cstdio>

int main() {
    std::printf("Dashboard feed test: spatial-ui pipeline headless...\n");

    exd::ecs::Registry registry;
    exd::scene_renderer::DocumentLoader loader{registry};
    exd::sim::DashboardFeedSystem feed{loader};

    // Seed a finished solver run + engine run so both feeds have data.
    auto run_entity = registry.create("SeedRun");
    exd::sim::SolveRunState s;
    s.status = exd::sim::SolveRunState::Done;
    s.final_cp = 0.42f;
    s.final_tsr = 5.2f;
    s.final_power_kw = 12.3f;
    s.trace_count = 32;
    for (int i = 0; i < s.trace_count; ++i)
        s.power_kw_trace[i] = 5.0f + 0.2f * static_cast<float>(i);
    registry.emplace<exd::sim::SolveRunState>(run_entity, s);

    auto eng_entity = registry.create("SeedEngine");
    exd::sim::EngineRunState e;
    e.status = exd::sim::EngineRunState::Done;
    e.mean_power_w = 9050.0;
    e.mean_omega_rad_s = 273.0;
    e.efficiency = 0.125;
    registry.emplace<exd::sim::EngineRunState>(eng_entity, e);
    exd::sim::IndicatorRecord ind;
    ind.count = 64;
    for (int i = 0; i < ind.count; ++i) {
        ind.crank_deg[i] = static_cast<float>(i) * 5.625f;
        ind.pressure_kpa[i] = 200.0f + 300.0f * static_cast<float>(i % 16);
        ind.volume_litres[i] = 0.1f + 0.8f * static_cast<float>(i % 7);
    }
    registry.emplace<exd::sim::IndicatorRecord>(eng_entity, ind);

    // Feed: builds the document + writes chart/text content.
    feed.update(registry, 0.0);
    feed.update(registry, 0.0);   // second pass: content unchanged → no dirty

    const auto chart_ent = loader.entity_for("dash.power_chart");
    if (!chart_ent) {
        std::printf("FAIL: chart node not materialized\n");
        return 1;
    }
    if (!registry.template has<exd::scene_renderer::MeshDirty>(*chart_ent)) {
        std::printf("FAIL: chart node not tagged dirty after feed\n");
        return 1;
    }

    // scene_renderer resolution (headless mesh injection).
    exd::scene_renderer::SizeSystem size;
    size.update(registry, 0.0);
    exd::scene_renderer::LayoutSystem layout{loader.hierarchy()};
    layout.update(registry, 0.0);
    int uploaded = 0;
    exd::scene_renderer::MeshSystem mesh(
        [&uploaded](const exd::core::MeshData&) {
            ++uploaded;
            return static_cast<uint32_t>(uploaded);
        });
    mesh.update(registry, 0.0);

    if (uploaded == 0) {
        std::printf("FAIL: no widget meshes produced\n");
        return 1;
    }
    const auto& renderable =
        registry.template get<exd::render::UIRenderableComponent>(*chart_ent);
    if (renderable.mesh_handle == 0) {
        std::printf("FAIL: chart entity has no mesh handle\n");
        return 1;
    }
    const auto queue = exd::scene_renderer::build_render_queue(registry);
    std::printf("OK: %d widget meshes uploaded, %zu UI draw entries, "
                "chart mesh=%u\n", uploaded, queue.size(), renderable.mesh_handle);
    return 0;
}
