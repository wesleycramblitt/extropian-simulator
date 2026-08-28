// OptimizationSystem implementation — embedded CMA-ES over the turbine
// design variables, evaluated with a physics-based objective that
// includes Reynolds-number-dependent aerodynamics and structural cost,
// so one generation completes per frame.
//
// All inter-system coupling is registry data flow:
//   reads/writes  OptimizationConfig + FitnessRecord  on the study entity
//   writes        TurbineSpec                        on the turbine entity
#include <exd/sim/optimization_system.hpp>

#include <exd/ecs/view.hpp>

#include <exd/render/systems/imgui_system.hpp>
#include <exd/sim/components/turbine.hpp>

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

namespace exd::sim {

namespace {

// ── Run configuration ────────────────────────────────────────────────
constexpr size_t kNumDesignVars = kOptimizationDesignVars; // design vector length
constexpr size_t kMaxEvaluations = 200; // evaluation budget per run
constexpr uint64_t kSeed = 42;          // deterministic seeded run
constexpr size_t kNumThreads = 1;       // sequential (default λ = 10 at 8D)

// ── Physics-based turbine objective ──────────────────────────────────
//
// Uses the actuator-disk / BEMT-inspired model:
//
//   Power  = ½ ρ A v³ Cp(λ, blade_geo)
//
// where:
//   A   = π R²                    (swept area)
//   v   = wind speed              (environmental)
//   Cp  = power coefficient       (function of tip-speed ratio & blade geo)
//   λ   = Ω R / v                (tip-speed ratio; Ω from RPM)
//
// Cp is modelled via the Schmitz/Glauert optimum for a given λ, degraded
// by:
//   • Reynolds-number drag penalty  (Re = ρ v R / μ)
//   • Blade solidity / chord mismatch
//   • Thickness-to-chord structural effects
//
// Structural blade mass:
//   m_blade ≈ ρ_mat × R × c_avg × t_avg × K_shape
//
// Objective = -(Annual Energy Proxy × Revenue − Blade Cost)
//   Revenue  = Power × capacity_factor
//   Cost     = n_blades × m_blade × cost_per_kg
//
// This creates a real tradeoff: bigger blades capture more energy but
// cost more, and the optimal size depends on wind speed and viscosity.

/// Aerodynamic efficiency factor for Reynolds number.
/// At low Re the lift-to-drag ratio degrades; this models the Cp reduction.
/// Re_full is the chord-based Reynolds number at the blade tip.
double reynolds_efficiency(double Re_full) {
    // Smooth ramp: below Re=1e5 efficiency drops sharply, above Re=5e5 it's near 1.0
    // Using a logistic curve centred at Re=2e5 with width factor 0.3 decades
    const double log_re = std::log10(std::max(Re_full, 1.0));
    const double midpoint = 5.0;   // log10(1e5) = 5.0
    const double width    = 0.6;
    return 0.4 + 0.6 / (1.0 + std::exp(-(log_re - midpoint) / width));
}

/// Schmitz optimum Cp for a given tip-speed ratio λ.
/// Approximates the Betz/Schmitz optimum:  Cp_max ≈ 16/(27) × f(λ)
/// with  f(λ) = (1 − 1.43/λ²) for moderate λ, capped at Betz limit.
double schmitz_cp(double lambda) {
    if (lambda < 0.5) return 0.0;
    // Schmitz approximation for optimum rotor
    const double lam2 = lambda * lambda;
    const double cp = (16.0 / 27.0) * lambda * std::pow(1.0 - 1.386 / lam2, 2);
    return std::clamp(cp, 0.0, 16.0 / 27.0);  // never exceed Betz limit
}

double evaluate_turbine_design(const std::vector<double>& eng,
                               double wind_speed, double viscosity,
                               double air_density, double hub_height,
                               double cost_per_kg, double blade_density) {
    if (eng.size() < kNumDesignVars)
        return std::numeric_limits<double>::infinity();

    const double radius     = eng[0];  //  1..8       [m]
    const double hub_radius = eng[1];  //  0.2..1.0   [m]
    const double chord      = eng[2];  //  0.3..2.5   [m]
    const double tip_taper  = eng[3];  //  0.35..1.0  [-]
    const double pitch      = eng[4];  // -180..180   [deg]
    const double twist      = eng[5];  // -90..90     [deg]
    const double thickness  = eng[6];  //  0.06..0.35 [t/c]
    const double hub_nose   = eng[7];  //  0..3       [m]
    (void)tip_taper;
    (void)hub_nose;

    // ── Wind shear: wind speed at hub height (power-law profile, α≈0.14) ──
    const double v_ref = wind_speed;                    // at 10 m reference
    const double v_hub = v_ref * std::pow(hub_height / 10.0, 0.14);

    // ── Swept area ──
    const double swept_area = std::numbers::pi_v<double> * radius * radius;

    // ── Reynolds number (chord-based, at blade tip) ──
    const double Re = air_density * v_hub * chord / viscosity;

    // ── Aerodynamic efficiency ──
    const double re_eff = reynolds_efficiency(Re);

    // ── Tip-speed ratio (RPM → rad/s, pick a reasonable Ω for the Cp model) ──
    //    Optimal TSR for a 3-blade HAWT ≈ 6–8; use design-optimal Ω.
    //    Ω = λ_opt × v_hub / R, with λ_opt ≈ 7
    const double omega_opt = 7.0 * v_hub / std::max(radius, 0.1);
    const double tsr = omega_opt * radius / std::max(v_hub, 0.1);

    // ── Power coefficient from Schmitz optimum ──
    double cp = schmitz_cp(tsr);

    // ── Pitch penalty: deviation from optimal pitch for this TSR ──
    const double pitch_opt = 2.0 + 0.5 * (tsr - 6.0); // pitch shifts with TSR
    const double pitch_err = (pitch - pitch_opt) * (pitch - pitch_opt);
    cp *= std::exp(-pitch_err / 200.0);  // gentle Gaussian penalty

    // ── Twist penalty: optimal twist ramp ≈ 14°/R for HAWTs ──
    const double twist_opt = 14.0 * radius / std::max(chord, 0.1);
    const double twist_clamped = std::clamp(twist_opt, 5.0, 40.0);
    const double twist_err = (twist - twist_clamped) * (twist - twist_clamped);
    cp *= std::exp(-twist_err / 400.0);

    // ── Thickness penalty: 10–18% is good for structural + aero ──
    double thick_eff = 1.0;
    if (thickness < 0.08)  thick_eff = thickness / 0.08;
    if (thickness > 0.22)  thick_eff = 0.22 / thickness;
    cp *= thick_eff;

    // ── Hub-to-tip ratio penalty ──
    const double htr = hub_radius / std::max(radius, 1e-9);
    const double annulus_loss = 1.0 - htr * htr;  // fraction of area that's useful
    cp *= annulus_loss;

    // ── Solidity / chord matching ──
    //    Optimal solidity σ = n·c/(2πR) for a given λ
    const double solidity = 3.0 * chord / (2.0 * std::numbers::pi_v<double> * radius);
    const double sigma_opt = 0.02 + 0.8 / std::max(tsr, 1.0);
    const double sigma_err = (solidity - sigma_opt) / std::max(sigma_opt, 0.01);
    cp *= std::exp(-sigma_err * sigma_err * 2.0);

    // Clamp Cp to Betz limit
    cp = std::clamp(cp, 0.0, 16.0 / 27.0);

    // ── Mechanical + electrical losses (gearbox, generator, drivetrain) ──
    const double drivetrain_eff = 0.88;  // typical geared HAWT

    // ── Available wind power × Cp × drivetrain × Re efficiency ──
    const double power = 0.5 * air_density * swept_area * std::pow(v_hub, 3)
                         * cp * drivetrain_eff * re_eff;

    // ── Blade structural mass ──
    //    Approximate blade as a tapered beam: m ≈ ρ_mat × L × c_avg × t_avg × K
    const double c_avg = chord * (1.0 + tip_taper) * 0.5;
    const double t_avg = c_avg * thickness;
    const double blade_span = radius - hub_radius;
    const double shape_factor = 0.65;  // taper + airfoil shape factor
    const double blade_mass = blade_density * blade_span * c_avg * t_avg * shape_factor;
    const double total_blade_cost = 3.0 * blade_mass * cost_per_kg;  // 3 blades

    // ── Hub cost (scales with hub radius³) ──
    const double hub_cost = 50.0 * hub_radius * hub_radius * hub_radius;

    // ── Objective: maximize (power revenue − cost) ──
    //    Revenue proxy: power × 10-yr capacity factor (CF ≈ 0.25–0.45)
    //    Normalize to reasonable MW-scale numbers.
    const double capacity_factor = 0.30;  // typical onshore
    const double lifetime_hours = 8760.0 * 10.0;  // 10 years
    const double revenue = power * 1e-6 * capacity_factor * lifetime_hours;  // in MW·h units
    const double total_cost = (total_blade_cost + hub_cost) * 1e-3;  // normalized

    const double objective = total_cost - revenue;  // minimize → maximize net value

    return objective;
}

/// Map an engineering design vector into the turbine's TurbineSpec.
void apply_params(TurbineSpec& p, const std::vector<double>& eng) {
    p.radius      = static_cast<float>(eng[0]);
    p.hub_radius  = static_cast<float>(eng[1]);
    p.axial_chord = static_cast<float>(eng[2]);
    p.tip_taper   = static_cast<float>(eng[3]);
    p.pitch_deg   = static_cast<float>(eng[4]);
    p.twist_deg   = static_cast<float>(eng[5]);
    p.thickness   = static_cast<float>(eng[6]);
    p.hub_nose    = static_cast<float>(eng[7]);
}

/// True when the handle was set (i.e. not the default-constructed Entity).
bool entity_set(ecs::Entity e) {
    return e.id != std::numeric_limits<ecs::Entity::id_type>::max();
}

} // namespace

// ── Accessors ───────────────────────────────────────────────────────

size_t OptimizationSystem::generation() const {
    return optimizer_ ? optimizer_->generation() : current_generation_;
}

size_t OptimizationSystem::evaluations() const {
    return optimizer_ ? optimizer_->evaluations() : current_evaluations_;
}

// ── Entities ────────────────────────────────────────────────────────

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

void OptimizationSystem::find_turbine(ecs::Registry& registry) {
    if (entity_set(turbine_) && registry.valid(turbine_)) return;  // already resolved
    registry.view<TurbineSpec>().each(
        [this](ecs::Entity e, const TurbineSpec&) { if (!entity_set(turbine_)) turbine_ = e; });
}

// ── Start / stop ────────────────────────────────────────────────────

void OptimizationSystem::start_optimization() {
    if (running_) return;

    // 8 design variables with engineering-space bounds.
    exd::opt::Problem problem;
    problem.variables.push_back({.lower = 1.0,   .upper = 8.0});    // radius     [m]
    problem.variables.push_back({.lower = 0.2,   .upper = 1.0});    // hub_radius [m]
    problem.variables.push_back({.lower = 0.3,   .upper = 2.5});    // axial_chord [m]
    problem.variables.push_back({.lower = 0.35,  .upper = 1.0});    // tip_taper  [-]
    problem.variables.push_back({.lower = -180.0, .upper = 180.0}); // pitch_deg  [deg]
    problem.variables.push_back({.lower = -90.0,  .upper = 90.0});  // twist_deg  [deg]
    problem.variables.push_back({.lower = 0.06,  .upper = 0.35});   // thickness  [t/c]
    problem.variables.push_back({.lower = 0.0,   .upper = 3.0});    // hub_nose   [m]

    exd::opt::OptimizeOptions opts;
    opts.max_evaluations = kMaxEvaluations;
    opts.seed = kSeed;
    opts.n_threads = kNumThreads;
    opts.track_history = true;

    // ── Seed from the current turbine design so CMA-ES iterates on it ──
    if (reg_ && entity_set(turbine_) && reg_->valid(turbine_)) {
        if (const auto* spec = reg_->try_get<TurbineSpec>(turbine_)) {
            const double eng[] = {
                spec->radius, spec->hub_radius, spec->axial_chord, spec->tip_taper,
                spec->pitch_deg, spec->twist_deg, spec->thickness, spec->hub_nose
            };
            opts.initial_mean.resize(kNumDesignVars);
            for (size_t i = 0; i < kNumDesignVars; ++i) {
                const double lo = problem.variables[i].lower;
                const double hi = problem.variables[i].upper;
                opts.initial_mean[i] = std::clamp((eng[i] - lo) / (hi - lo), 0.0, 1.0);
            }
        }
    }

    optimizer_ = std::make_unique<exd::opt::Optimizer>(std::move(problem),
                                                       exd::opt::Algo::CMAES,
                                                       std::move(opts));
    result_ = exd::opt::OptimizationResult{};
    current_generation_ = 0;
    current_evaluations_ = 0;
    current_best_fitness_ = std::numeric_limits<double>::infinity();

    // Seed the display with the current turbine design
    if (reg_ && entity_set(turbine_) && reg_->valid(turbine_)) {
        if (const auto* spec = reg_->try_get<TurbineSpec>(turbine_)) {
            best_design_ = {
                spec->radius, spec->hub_radius, spec->axial_chord, spec->tip_taper,
                spec->pitch_deg, spec->twist_deg, spec->thickness, spec->hub_nose
            };
        }
    } else {
        best_design_.clear();
    }
    running_ = true;
    publish_fitness();

    const auto& cfg = reg_->get<OptimizationConfig>(study_);
    std::printf("[Optimization] CMA-ES started: v=%.1f m/s, μ=%.1e m²/s, "
                "ρ=%.3f kg/m³, %zu design vars, %zu evals\n",
                cfg.wind_speed, cfg.viscosity, cfg.air_density,
                optimizer_->problem().dim(), kMaxEvaluations);
}

void OptimizationSystem::stop_optimization() {
    running_ = false;
    if (optimizer_) optimizer_->request_stop();
    if (reg_) publish_fitness();
}

// ── Frame ───────────────────────────────────────────────────────────

void OptimizationSystem::update(ecs::Registry& registry, double dt) {
    (void)dt;
    reg_ = &registry;
    ensure_entities(registry);
    find_turbine(registry);
    if (!running_ || !optimizer_) return;

    // 1. Pull the next batch of candidate designs.
    std::vector<exd::opt::design> batch = optimizer_->request_batch();

    const OptimizationConfig& cfg = registry.get<OptimizationConfig>(study_);

    // 2. An empty batch means the run is finished — finalize.
    if (batch.empty()) {
        running_ = false;
        result_ = optimizer_->result();
        if (!result_.best_x.empty()) {
            best_design_ = result_.best_x;
            if (!result_.best_fitness.empty())
                current_best_fitness_ = result_.best_fitness[0];
        }
        current_generation_ = optimizer_->generation();
        current_evaluations_ = optimizer_->evaluations();
        apply_best_design();
        publish_fitness();
        std::printf("[Optimization] finished: %s, %zu evals, %zu generations\n",
                    exd::opt::to_string(result_.stop_reason),
                    current_evaluations_, current_generation_);
        return;
    }

    // 3. Evaluate every candidate inline (sequential, one batch per frame).
    std::vector<exd::opt::Evaluation> evals;
    evals.reserve(batch.size());
    for (const auto& cand : batch) {
        const std::vector<double> eng =
            exd::opt::to_engineering(optimizer_->problem(), cand);
        const double f = evaluate_turbine_design(
            eng, cfg.wind_speed, cfg.viscosity, cfg.air_density,
            cfg.hub_height, cfg.cost_per_kg, cfg.blade_density);
        if (f < current_best_fitness_) {
            current_best_fitness_ = f;
            best_design_ = eng;
        }
        evals.push_back(exd::opt::Evaluation{{f}});
    }

    // 4. Hand the results back; the optimizer steps a generation when the
    //    whole population has been submitted.
    optimizer_->submit_results(std::move(evals));

    // 5. If the step ended the run (convergence / budget hit), keep the
    //    engine's final verdict as the authoritative answer.
    if (!optimizer_->running()) {
        running_ = false;
        result_ = optimizer_->result();
        if (!result_.best_x.empty()) {
            best_design_ = result_.best_x;
            if (!result_.best_fitness.empty())
                current_best_fitness_ = result_.best_fitness[0];
        }
    }

    // 6. Refresh display stats.
    current_generation_ = optimizer_->generation();
    current_evaluations_ = optimizer_->evaluations();

    // 7. Reflect the best-so-far design on the live turbine.
    apply_best_design();
    publish_fitness();
}

// ── Turbine driving (ECS data flow) ─────────────────────────────────

void OptimizationSystem::apply_best_design() {
    if (!reg_ || best_design_.size() < kNumDesignVars) return;
    if (!entity_set(turbine_) || !reg_->valid(turbine_)) return;
    // The turbine is located by its component, not by a system reference:
    // write TurbineSpec and TurbineSystem picks it up on its next update.
    if (auto* spec = reg_->try_get<TurbineSpec>(turbine_))
        apply_params(*spec, best_design_);
}

void OptimizationSystem::publish_fitness() {
    if (!reg_ || !entity_set(study_) || !reg_->valid(study_)) return;
    auto& rec = reg_->get<FitnessRecord>(study_);
    rec.running = running_;
    rec.generation = static_cast<int>(generation());
    rec.evaluations = static_cast<int>(evaluations());
    rec.best_fitness = current_best_fitness_;
    for (size_t i = 0; i < best_design_.size() && i < kNumDesignVars; ++i)
        rec.best_design[i] = static_cast<float>(best_design_[i]);
}

// ── Panel ───────────────────────────────────────────────────────────

const char* OptimizationSystem::status_text() const {
    if (running_) return "Running...";
    if (!optimizer_ || optimizer_->evaluations() == 0) return "Idle";
    if (optimizer_->running()) return "Stopped by user";
    switch (result_.stop_reason) {
    case exd::opt::StopReason::ConvergedPlateau:
    case exd::opt::StopReason::ConvergedTolerance:
        return "Converged";
    case exd::opt::StopReason::UserAbort:
        return "Stopped by user";
    case exd::opt::StopReason::MaxEvaluations:
    case exd::opt::StopReason::MaxGenerations:
    case exd::opt::StopReason::MaxWallTime:
    case exd::opt::StopReason::AlgorithmTerminated:
        return "Finished";
    }
    return "Finished";
}

void OptimizationSystem::draw_panel() {
    if (!reg_ || !entity_set(study_) || !reg_->valid(study_)) return;
    auto& cfg = reg_->get<OptimizationConfig>(study_);
    const auto& rec = reg_->get<FitnessRecord>(study_);

    ImGui::Text("CMA-ES turbine design optimization");
    ImGui::Separator();

    // ── Environment settings (stored in OptimizationConfig on the study) ──
    ImGui::Text("Environment");
    ImGui::SliderFloat("Wind speed [m/s]", &cfg.wind_speed, 1.0f, 40.0f, "%.1f");
    ImGui::SliderFloat("Viscosity [m²/s]", &cfg.viscosity, 5.0e-6f, 5.0e-4f, "%.1e");
    ImGui::SliderFloat("Air density [kg/m³]", &cfg.air_density, 0.5f, 1.5f, "%.3f");
    ImGui::SliderFloat("Hub height [m]", &cfg.hub_height, 5.0f, 100.0f, "%.0f");
    ImGui::Separator();
    ImGui::SliderFloat("Cost per kg", &cfg.cost_per_kg, 0.1f, 10.0f, "%.1f");
    ImGui::SliderFloat("Blade density [kg/m³]", &cfg.blade_density, 500.0f, 3000.0f, "%.0f");

    ImGui::Separator();

    // ── Start / stop ──
    if (running_) {
        if (ImGui::Button("Stop Optimization")) stop_optimization();
    } else {
        if (ImGui::Button("Start Optimization")) start_optimization();
    }

    ImGui::Separator();
    ImGui::Text("Status: %s", status_text());
    ImGui::Text("Generation: %d", rec.generation);
    ImGui::Text("Evaluations: %d / %zu", rec.evaluations, kMaxEvaluations);
    if (std::isfinite(rec.best_fitness)) {
        ImGui::Text("Best fitness: %.4f", rec.best_fitness);
    } else {
        ImGui::Text("Best fitness: --");
    }

    ImGui::Separator();
    ImGui::Text("Best design so far");
    if (rec.evaluations > 0) {
        ImGui::Text("Radius:       %.2f m",    rec.best_design[0]);
        ImGui::Text("Hub radius:   %.2f m",    rec.best_design[1]);
        ImGui::Text("Axial chord:  %.2f m",    rec.best_design[2]);
        ImGui::Text("Tip taper:    %.2f",      rec.best_design[3]);
        ImGui::Text("Pitch:        %.1f deg",  rec.best_design[4]);
        ImGui::Text("Twist:        %.1f deg",  rec.best_design[5]);
        ImGui::Text("Thickness:    %.3f t/c",  rec.best_design[6]);
        ImGui::Text("Hub nose:     %.2f m",    rec.best_design[7]);
    } else {
        ImGui::Text("(none yet)");
    }

    ImGui::Separator();
    ImGui::TextWrapped(
        "Objective: maximize (power revenue − blade cost). "
        "Power uses actuator-disk model with Schmitz Cp, "
        "Reynolds-number efficiency, and drivetrain losses. "
        "Blade mass scales with R × c × t × density. "
        "Adjust wind speed, viscosity, and cost to see how the "
        "optimal design changes. One generation per frame.");
}

} // namespace exd::sim
