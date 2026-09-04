// OptimizationSystem implementation — generic embedded optimizer driver.
//
// All inter-system coupling is registry data flow:
//   reads/writes  OptimizationConfig + FitnessRecord  on the study entity
// The objective (std::function) is caller-injected construction state —
// system-local execution logic, not a cross-system API.
#include <exd/sim/optimization_system.hpp>

#include <exd/ecs/view.hpp>

#include <exd/render/systems/imgui_system.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <limits>
#include <utility>

namespace exd::sim {

namespace {

/// Clamp the panel-editable algo index to the exd::opt::Algo enum.
exd::opt::Algo to_algo(int i) {
    if (i < static_cast<int>(exd::opt::Algo::CMAES) ||
        i > static_cast<int>(exd::opt::Algo::DifferentialEvolution))
        return exd::opt::Algo::CMAES;
    return static_cast<exd::opt::Algo>(i);
}

} // namespace

// ── Entities ─────────────────────────────────────────────────────────

void OptimizationSystem::ensure_entities(ecs::Registry& registry) {
    if (entities_ready_) return;

    study_ = registry.create("OptimizationStudy");
    registry.emplace<OptimizationConfig>(study_);
    registry.emplace<FitnessRecord>(study_);

    if (!panel_added_) {
        auto panel = registry.create("OptimizationPanel");
        registry.emplace<render::ImGuiPanelComponent>(panel, "Optimization",
            [this] { draw_panel(); });
        panel_added_ = true;
    }

    entities_ready_ = true;
}

// ── Start / stop ────────────────────────────────────────────────────

void OptimizationSystem::start_optimization() {
    if (running_ || !reg_ || !objective_) return;
    ++run_counter_;

    const OptimizationConfig& cfg = reg_->get<OptimizationConfig>(study_);
    const int n = std::clamp(cfg.n_vars, 1, kOptimizationDesignVars);

    // Problem: n variables with engineering-space bounds. exd::opt works
    // in a unit hypercube internally and maps candidates back for us.
    exd::opt::Problem problem;
    for (int i = 0; i < n; ++i) {
        const float lo = std::min(cfg.lower[i], cfg.upper[i]);
        const float hi = std::max(cfg.lower[i], cfg.upper[i]);
        problem.variables.push_back({.lower = lo, .upper = hi});
    }

    exd::opt::OptimizeOptions options;
    options.max_evaluations = static_cast<size_t>(std::max(cfg.max_evaluations, 1));
    options.seed = cfg.seed + run_counter_;   // fresh deterministic run
    options.n_threads = 1;                    // embedded mode: host evaluates

    optimizer_ = std::make_unique<exd::opt::Optimizer>(
        std::move(problem), to_algo(cfg.algo), options);
    running_ = true;

    // Fresh per-run record: this run reports its own best.
    current_best_fitness_ = std::numeric_limits<double>::infinity();
    best_design_.clear();
    current_generation_ = 0;
    current_evaluations_ = 0;

    publish_fitness();
    std::printf("[Optimization] started: %s, %d vars, %zu evaluations\n",
                exd::opt::algorithm_name(to_algo(cfg.algo)), n,
                options.max_evaluations);
}

void OptimizationSystem::stop_optimization() {
    if (!running_ || !reg_) return;
    running_ = false;
    result_ = optimizer_ ? optimizer_->result() : exd::opt::OptimizationResult{};
    publish_fitness();
    std::printf("[Optimization] stopped by user.\n");
}

// ── Frame loop ──────────────────────────────────────────────────────

void OptimizationSystem::update(ecs::Registry& registry, double) {
    reg_ = &registry;
    ensure_entities(registry);
    if (!running_ || !optimizer_) return;

    // 1. Pull the next batch of candidate designs.
    std::vector<exd::opt::design> batch = optimizer_->request_batch();

    // 2. An empty batch means the run is finished — finalize.
    if (batch.empty()) {
        finalize_run();
        return;
    }

    // 3. Evaluate every candidate inline (one batch per frame).
    std::vector<exd::opt::Evaluation> evals;
    evals.reserve(batch.size());
    for (const auto& cand : batch) {
        const std::vector<double> eng =
            exd::opt::to_engineering(optimizer_->problem(), cand);
        exd::opt::Evaluation e = objective_(eng);
        if (!e.fitness.empty() && e.fitness[0] < current_best_fitness_) {
            current_best_fitness_ = e.fitness[0];
            best_design_ = eng;
        }
        evals.push_back(std::move(e));
    }

    // 4. Hand the results back; the optimizer steps a generation when the
    //    whole population has been submitted.
    optimizer_->submit_results(std::move(evals));

    // 5. If the step ended the run (convergence / budget hit), finalize.
    if (!optimizer_->running()) {
        finalize_run();
        return;
    }

    // 6. Refresh display stats.
    current_generation_ = optimizer_->generation();
    current_evaluations_ = optimizer_->evaluations();
    publish_fitness();
}

void OptimizationSystem::finalize_run() {
    running_ = false;
    result_ = optimizer_ ? optimizer_->result() : exd::opt::OptimizationResult{};
    // Adopt the engine's verdict, guarded by the run's record.
    if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
        result_.best_fitness[0] < current_best_fitness_) {
        best_design_ = result_.best_x;
        current_best_fitness_ = result_.best_fitness[0];
    }
    current_generation_ = optimizer_ ? optimizer_->generation() : 0;
    current_evaluations_ = optimizer_ ? optimizer_->evaluations() : 0;
    publish_fitness();
    std::printf("[Optimization] finished: %s, %zu evals, %zu generations, "
                "best f = %.6e\n",
                exd::opt::to_string(result_.stop_reason),
                current_evaluations_, current_generation_, current_best_fitness_);
}

// ── Publication + panel ─────────────────────────────────────────────

void OptimizationSystem::publish_fitness() {
    if (!reg_ || !reg_->valid(study_)) return;
    auto& rec = reg_->get<FitnessRecord>(study_);
    rec.best_fitness = current_best_fitness_;
    for (int i = 0; i < kOptimizationDesignVars; ++i)
        rec.best_design[i] = static_cast<float>(
            i < static_cast<int>(best_design_.size()) ? best_design_[i] : 0.0);
    rec.generation   = static_cast<int>(current_generation_);
    rec.evaluations  = static_cast<int>(current_evaluations_);
    rec.evals_pending = 0;
    rec.running      = running_;
}

const char* OptimizationSystem::status_text() const {
    if (!running_) {
        if (result_.ok()) return "finished";
        if (run_counter_ > 0) return "idle";
        return "idle — never run";
    }
    return "running";
}

void OptimizationSystem::draw_panel() {
    if (!reg_ || !entities_ready_) return;
    auto& cfg = reg_->get<OptimizationConfig>(study_);
    const auto& rec = reg_->get<FitnessRecord>(study_);

    ImGui::Text("Generic study — caller-provided objective");
    ImGui::TextDisabled("Best candidates arrive in engineering units; the "
                        "domain maps FitnessRecord back into its own state.");
    ImGui::Separator();

    // Algorithm selection from the exd::opt enum.
    const int n_algos = static_cast<int>(exd::opt::Algo::DifferentialEvolution) + 1;
    const char* algo_name = exd::opt::algorithm_name(to_algo(cfg.algo));
    if (ImGui::BeginCombo("Algorithm", algo_name)) {
        for (int i = 0; i < n_algos; ++i) {
            const bool selected = (cfg.algo == i);
            if (ImGui::Selectable(exd::opt::algorithm_name(
                                      static_cast<exd::opt::Algo>(i)),
                                  selected))
                cfg.algo = i;
        }
        ImGui::EndCombo();
    }

    ImGui::SliderInt("Variables", &cfg.n_vars, 1, kOptimizationDesignVars);
    for (int i = 0; i < cfg.n_vars && i < kOptimizationDesignVars; ++i) {
        ImGui::PushID(i);
        ImGui::DragFloat2("bounds", &cfg.lower[i], 0.01f, -1e6f, 1e6f, "%.3f");
        ImGui::PopID();
    }
    ImGui::InputInt("Max evaluations", &cfg.max_evaluations);
    if (cfg.max_evaluations < 1) cfg.max_evaluations = 1;

    ImGui::Separator();
    if (running_) {
        if (ImGui::Button("Stop Optimization")) stop_optimization();
    } else {
        if (ImGui::Button("Start Optimization")) start_optimization();
    }
    ImGui::Text("Status: %s", status_text());
    ImGui::Text("Generation: %zu   Evaluations: %zu",
                current_generation_, current_evaluations_);
    ImGui::Text("Best fitness: %.6e", rec.best_fitness);

    if (rec.evaluations > 0) {
        ImGui::Separator();
        ImGui::TextDisabled("Best design (engineering units)");
        ImGui::Text("  [");
        for (int i = 0; i < cfg.n_vars && i < kOptimizationDesignVars; ++i) {
            if (i) ImGui::SameLine();
            ImGui::Text("%.4f%s", rec.best_design[i],
                        i + 1 < cfg.n_vars ? "," : "");
        }
        ImGui::Text("  ]");
    }
}

} // namespace exd::sim
