#pragma once
// ─────────────────────────────────────────────────────────────────────
// SteamEngineSystem — 0D steam engine: meshing, simulation, animation.
//
//   • mesh: parametric exd::geometry::generate_steam_engine() regenerated
//     per crank angle (dirty-check throttled, TurbineSystem pattern)
//   • physics: exd::engine::presets::engine::simulate_engine() (Rankine-lite)
//     on a worker thread; results → EngineRunState + IndicatorRecord
//   • animation: crank advances at the last run's mean ω (scaled) or a
//     fixed idle pace; the mesh rebuilds when the angle crosses a step
//
// ECS contract (see AGENTS.md):
//   • owns a "SteamEngine" entity carrying EngineSpec + EngineRunState +
//     IndicatorRecord (+ render components for the machine mesh)
//   • reads/writes nothing else; OptimizationSystem (engine mode) writes
//     EngineSpec on the same entity between runs
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/sim/components/engine.hpp>

#include <memory>
#include <string>

namespace exd::sim {

class SteamEngineSystem final : public ecs::ISystem {
public:
    explicit SteamEngineSystem(render::GraphicsContext& ctx);
    ~SteamEngineSystem() override;   // defined in the .cpp (worker channel pimpl)

    void update(ecs::Registry& registry, double dt) override;

    // Panel action: snapshot the design and run the engine simulator.
    void start_run();
    bool running() const;

    void draw_panel();

private:
    void ensure_entities(ecs::Registry& registry);
    void rebuild_mesh(ecs::Registry& registry);
    void poll_worker(ecs::Registry& registry);

    // Worker channel (pimpl: std::future requires the complete payload type
    // at destruction, which only the .cpp can know via src/engine_run.hpp).
    struct Worker;
    std::unique_ptr<Worker> worker_;

    ecs::Registry* reg_ = nullptr;
    ecs::Entity entity_ = {};        // "SteamEngine"
    ecs::Entity panel_ = {};
    bool entities_ready_ = false;
    bool panel_added_ = false;

    render::GraphicsContext& ctx_;
    uint32_t mesh_handle_ = 0;

    EngineSpec last_spec_{};
    EngineSpec spec_snapshot_{};     // design the running sim is solving
    double crank_deg_ = 0.0;
    double last_rebuilt_deg_ = -1e9;
    std::string last_error_ = "none";
};

} // namespace exd::sim
