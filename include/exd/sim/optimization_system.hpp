#pragma once
// ─────────────────────────────────────────────────────────────────────
// OptimizationSystem — live CMA-ES turbine-design optimization driven
// from inside the frame loop.
//
// Every frame while a run is active the system pulls the next batch of
// candidate designs from exd::opt::Optimizer (embedded mode), evaluates
// them with a physics-based efficiency objective, hands the results back,
// and applies the best-so-far design to the turbine so the shape morphs
// as the optimizer progresses.
//
// ECS contract (see AGENTS.md):
//   • owns an "OptimizationStudy" entity carrying OptimizationConfig
//     (panel-editable environment) and FitnessRecord (published results)
//   • finds the turbine through the registry: entities carrying
//     TurbineSpec (first match wins) — writes TurbineSpec directly,
//     never through a TurbineSystem reference
//   • registers its own ImGui panel via ImGuiPanelComponent
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/opt/opt.hpp>
#include <exd/sim/components/engine.hpp>
#include <exd/sim/components/optimization.hpp>
#include <exd/sim/components/turbine.hpp>

#include <limits>
#include <atomic>
#include <future>
#include <memory>
#include <vector>

namespace exd::sim {

/// Objective model selection: analytic (fast inline objective, the default
/// demo) or coupled-CFD (one short background FDM3 run per candidate).
enum class ObjectiveModel : uint8_t {
    Analytic = 0,    // fast inline turbine objective (default demo)
    CoupledCfd = 1,  // one short background FDM3 run per candidate
    EngineSim = 2,   // fast inline 0D steam-engine objective
};

/// Outcome of one coupled-CFD candidate evaluation (worker payload).
struct CfdEvalResult {
    bool valid = false;
    double fitness = 0.0;   // minimize: −Cp, or a penalty for invalid runs
    std::string error;
    double cp = 0.0, tsr = 0.0, power_w = 0.0, wall_seconds = 0.0;
};

class OptimizationSystem final : public ecs::ISystem {
public:
    explicit OptimizationSystem(ObjectiveModel model = ObjectiveModel::Analytic)
        : model_(model) {}

    void update(ecs::Registry& registry, double dt) override;

    // Start / stop optimization (panel actions; the registry is bound
    // lazily on the first update, so these are safe to call after that).
    void start_optimization();
    void stop_optimization();

    // Accessors for ImGui display
    bool is_running() const { return running_; }
    const exd::opt::OptimizationResult& result() const { return result_; }
    size_t generation() const;
    size_t evaluations() const;

    // ImGui panel body (called by the panel entity's ImGuiPanelComponent).
    void draw_panel();

private:
    // Bind the registry and create the study/panel entities on first update.
    void ensure_entities(ecs::Registry& registry);
    // Locate the turbine entity(s) carrying TurbineSpec — first match wins.
    void find_turbine(ecs::Registry& registry);
    // Apply the best-so-far design to the turbine's TurbineSpec component.
    void apply_best_design();
    void publish_fitness();
    const char* status_text() const;
    // True when the live turbine design differs from the all-time record
    // (the user edited the Turbine panel between runs).
    bool turbine_tweaked() const;

    ecs::Registry* reg_ = nullptr;
    ecs::Entity turbine_ = {};        // first entity with TurbineSpec
    ecs::Entity study_ = {};          // OptimizationConfig + FitnessRecord
    bool entities_ready_ = false;

    std::unique_ptr<exd::opt::Optimizer> optimizer_;
    exd::opt::OptimizationResult result_;
    bool running_ = false;
    bool panel_added_ = false;

    // Current best design in engineering units
    std::vector<double> best_design_;

    // Optimization stats for display
    size_t current_generation_ = 0;
    size_t current_evaluations_ = 0;
    double current_best_fitness_ = std::numeric_limits<double>::infinity(); // all-time best (current env)

    // Cross-run bookkeeping — the record persists across runs for the
    // current environment, and every run draws a fresh deterministic
    // seed, so repeated "Start" clicks keep improving the record.
    uint64_t run_counter_ = 0;
    OptimizationConfig baseline_config_{};
    bool has_baseline_ = false;    // baseline (record) initialized at all
    bool completed_run_ = false;   // a run finished under baseline_config_

    // ── Coupled-CFD objective mode ──────────────────────────────────
    // Candidates are queued from the optimizer and evaluated ONE AT A TIME
    // on a worker thread (each is a short coupled FDM3 run); results are
    // submitted to the optimizer when the whole batch (λ) is complete.
    ObjectiveModel model_ = ObjectiveModel::Analytic;
    struct CfdEvalCfg {
        float wind_speed = 8.0f, ramp_time_s = 1.0f;
        int grid = 12, steps = 1500;
    };
    void cfd_update(ecs::Registry& registry);
    void engine_update(ecs::Registry& registry);
    void apply_best_engine_design();
    CfdEvalCfg cfd_cfg_{};
    EngineSpec engine_base_{};   // fixed fields snapshot for engine evals
    std::vector<exd::opt::design> pending_batch_;
    std::vector<double> pending_fitness_;
    size_t pending_done_ = 0;
    std::future<std::unique_ptr<CfdEvalResult>> cfd_future_;
    std::atomic<bool> cfd_busy_{false};
    uint64_t cfd_run_token_ = 0;   // run_counter_ at launch; stale results drop
};

} // namespace exd::sim
