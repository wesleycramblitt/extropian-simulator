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
#include <exd/sim/components/optimization.hpp>
#include <exd/sim/components/turbine.hpp>

#include <memory>
#include <vector>

namespace exd::sim {

class OptimizationSystem final : public ecs::ISystem {
public:
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
    double current_best_fitness_ = 0.0;
};

} // namespace exd::sim
