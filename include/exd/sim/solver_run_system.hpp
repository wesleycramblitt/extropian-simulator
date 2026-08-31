#pragma once
// ─────────────────────────────────────────────────────────────────────
// SolverRunSystem — one-shot real CFD run of the current turbine design.
//
// On "Solve" the system snapshots the current TurbineSpec + SolverRunConfig
// and launches exd::physics::turbine::run_coupled_turbine() on a worker
// thread (the solver never touches the registry). While it runs the panel
// shows progress; on completion the results land in the SolveRunState
// component on the "SolverRun" entity and the flow field is visualized:
//   • streamlines seeded upstream of the rotor, colored by speed
//   • an axial (rotor-plane) slice of speed magnitude as a colored quad mesh
// Both are published as render entities via the existing render pipeline.
//
// ECS contract (see AGENTS.md):
//   • owns a "SolverRun" entity carrying SolverRunConfig + SolveRunState
//   • finds the turbine through TurbineSpec (first match wins) and reads its
//     design at Solve time — never references TurbineSystem
//   • writes renderable entities for the flow field, not the turbine mesh
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/physics/fluid/fdm3/fdm3_result.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/sim/components/solver_run.hpp>
#include <exd/sim/components/turbine.hpp>

#include <atomic>
#include <future>
#include <string>
#include <vector>

namespace exd::sim {

class SolverRunSystem final : public ecs::ISystem {
public:
    explicit SolverRunSystem(render::GraphicsContext& ctx) : ctx_(ctx) {}

    void update(ecs::Registry& registry, double dt) override;

    // Panel action: snapshot the current design and launch a run.
    void solve();
    bool solving() const { return worker_running_.load(std::memory_order_relaxed); }

    void draw_panel();

public:
    /// Worker payload: a completed coupled run. Owned by the main thread
    /// after the worker finishes; never exposed as a component (vectors).
    struct RunResult {
        bool valid = false;
        std::string error;
        int nx = 0, ny = 0, nz = 0, steps_taken = 0;
        double final_cp = 0.0, final_tsr = 0.0, power_w = 0.0, wall_seconds = 0.0;
        std::vector<double> power_w_trace;   // exchange cadence, for dashboards
        exd::physics::fluid::fdm3::FDM3FieldData field;
    };

private:
    void ensure_entities(ecs::Registry& registry);
    void find_turbine(ecs::Registry& registry);
    void poll_worker(ecs::Registry& registry);
    void publish_viz(ecs::Registry& registry);

    ecs::Registry* reg_ = nullptr;
    ecs::Entity turbine_ = {};     // first entity with TurbineSpec
    ecs::Entity run_ = {};         // SolverRunConfig + SolveRunState
    ecs::Entity flow_slice_ = {};  // rotor-plane slice mesh entity
    ecs::Entity flow_lines_ = {};  // streamline mesh entity
    bool entities_ready_ = false;
    bool panel_added_ = false;

    render::GraphicsContext& ctx_;

    // Worker channel: one in-flight run at a time; the worker returns a
    // heap payload that the main thread adopts on completion.
    std::future<std::unique_ptr<RunResult>> future_;
    std::atomic<bool> worker_running_{false};
    std::unique_ptr<RunResult> result_;   // adopted on the main thread

    uint32_t slice_handle_ = 0;
    uint32_t lines_handle_ = 0;
    std::string last_error_ = "none";
};

} // namespace exd::sim
