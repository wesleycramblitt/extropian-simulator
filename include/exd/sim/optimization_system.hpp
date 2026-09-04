#pragma once
// ─────────────────────────────────────────────────────────────────────
// OptimizationSystem — generic frame-batched optimizer driver.
//
// The system is deliberately domain-agnostic:
//   • the objective is injected at construction
//     (std::function<Evaluation(const design&)>, candidates arrive in
//     ENGINEERING units bounded by OptimizationConfig lower/upper),
//   • the study definition (variables, bounds, algorithm, budget) lives
//     in the OptimizationConfig component, edited via the panel,
//   • results are published on the study entity as FitnessRecord; the
//     domain that owns the variables maps them back into its own
//     components (the workspace's CAD layer does that when it lands).
//
// Objective cost is the caller's choice: the system evaluates one batch
// per frame inline. Cheap analytic objectives complete in a frame or two;
// an expensive one (physics run per candidate) can be wrapped by the
// caller with its own background worker / cache — nothing here assumes
// evaluation cost.
//
// ECS contract (see AGENTS.md):
//   • owns an "OptimizationStudy" entity carrying OptimizationConfig
//     (panel-editable) and FitnessRecord (published results)
//   • registers its own ImGui panel via ImGuiPanelComponent
//   • communicates with other systems ONLY through components — never
//     references a domain system (turbine, engine, ...)
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/opt/opt.hpp>
#include <exd/sim/components/optimization.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace exd::sim {

class OptimizationSystem final : public ecs::ISystem {
public:
    /// Generic objective: design vector in engineering units -> fitness.
    using Objective = std::function<exd::opt::Evaluation(const exd::opt::design&)>;

    explicit OptimizationSystem(Objective objective)
        : objective_(std::move(objective)) {}

    void update(ecs::Registry& registry, double dt) override;

    // Start / stop the study (panel actions; the registry is bound
    // lazily on the first update, so these are safe to call after that).
    void start_optimization();
    void stop_optimization();

    // Accessors for ImGui display.
    bool is_running() const { return running_; }
    const exd::opt::OptimizationResult& result() const { return result_; }
    size_t generation() const { return current_generation_; }
    size_t evaluations() const { return current_evaluations_; }

    // ImGui panel body (called by the panel entity's ImGuiPanelComponent).
    void draw_panel();

private:
    // Bind the registry and create the study/panel entities on first update.
    void ensure_entities(ecs::Registry& registry);
    // Write FitnessRecord from the current run state.
    void publish_fitness();
    const char* status_text() const;
    // Adopt the optimizer's final verdict, guarded by the run's record.
    void finalize_run();

    ecs::Registry* reg_ = nullptr;
    ecs::Entity study_ = {};        // OptimizationConfig + FitnessRecord
    bool entities_ready_ = false;
    bool panel_added_ = false;

    Objective objective_;
    std::unique_ptr<exd::opt::Optimizer> optimizer_;
    exd::opt::OptimizationResult result_;
    bool running_ = false;

    // Current run's best design (engineering units) + all-time record.
    std::vector<double> best_design_;
    double current_best_fitness_ = std::numeric_limits<double>::infinity();

    // Run bookkeeping: stats for display + fresh deterministic seeds.
    size_t current_generation_ = 0;
    size_t current_evaluations_ = 0;
    uint64_t run_counter_ = 0;
};

} // namespace exd::sim
