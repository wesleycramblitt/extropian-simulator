// OptimizationSystem implementation — embedded CMA-ES over the turbine
// design variables, evaluated with a physics-based objective that
// includes Reynolds-number-dependent aerodynamics and structural cost,
// so one generation completes per frame.
//
// All inter-system coupling is registry data flow:
//   reads/writes  OptimizationConfig + FitnessRecord  on the study entity
//   writes        TurbineSpec                        on the turbine entity
#include <exd/sim/optimization_system.hpp>

#include "coupled_run.hpp"
#include "engine_run.hpp"

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
constexpr size_t kMaxEvaluations = 2000; // evaluation budget per run
                                          // (≈ 200 generations at λ=10 — long
                                          //  enough to visibly converge)
constexpr uint64_t kSeed = 42;          // base seed; each run gets a fresh
                                        // (deterministic) seed = kSeed + run#
constexpr size_t kNumThreads = 1;       // sequential (default λ = 10 at 8D)
constexpr double kInitialSigmaFull   = 0.3;  // first run: explore the box
constexpr double kInitialSigmaRefine = 0.05; // continuation: refine the record

// ── Coupled-CFD objective mode ──────────────────────────────────────
constexpr size_t kCfdMaxEvaluations = 60;   // 6 generations of λ=10
constexpr int    kCfdGrid   = 12;           // cells per axis (~1.7k cells)
constexpr int    kCfdSteps  = 1500;         // fluid steps (~0.7 s / candidate)
constexpr double kCfdRamp   = 1.0;          // ramp (floored by the recipe)
constexpr double kCfdPenalty = 1e3;         // fitness for invalid/unstable runs

// ── Engine-sim objective mode ───────────────────────────────────────
constexpr size_t kEngineMaxEvaluations = 400; // budget (evals are ~ms each)
constexpr double kEnginePenalty = 1e6;        // fitness for invalid designs

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

/// Worker for one coupled-CFD candidate: build a design snapshot, run the
/// short coupled FDM3 solver, return the objective. No registry access.
std::unique_ptr<CfdEvalResult> cfd_eval_worker(
    const TurbineSpec& design, float wind_speed, float ramp_time_s,
    int grid, int steps) {
    auto out = std::make_unique<CfdEvalResult>();
    const impl::CoupledRunOutcome outcome =
        impl::run_coupled_eval(design, wind_speed, grid, steps, ramp_time_s,
                               1.8, 3.0, nullptr);
    out->valid = outcome.valid;
    out->error = outcome.error;
    out->cp = outcome.final_cp;
    out->tsr = outcome.final_tsr;
    out->power_w = outcome.power_w;
    out->wall_seconds = outcome.wall_seconds;
    // Objective: minimize −Cp; invalid/unstable runs get a large penalty so
    // CMA-ES steers away from them.
    out->fitness = outcome.valid ? -outcome.final_cp : kCfdPenalty;
    return out;
}

/// Engineer an EngineSpec from the design vector + the fixed snapshot:
/// eng = {p_boiler, cutoff_deg, crank_radius, bore}.
void apply_engine_params(EngineSpec& p, const std::vector<double>& eng) {
    p.p_boiler    = static_cast<float>(eng[0]);
    p.cutoff_deg  = static_cast<float>(eng[1]);
    p.crank_radius = static_cast<float>(eng[2]);
    p.bore        = static_cast<float>(eng[3]);
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

/// Environment config comparison — the all-time best only stays valid
/// while the objective inputs are unchanged.
bool same_config(const OptimizationConfig& a, const OptimizationConfig& b) {
    return a.wind_speed == b.wind_speed && a.viscosity == b.viscosity &&
           a.air_density == b.air_density && a.hub_height == b.hub_height &&
           a.cost_per_kg == b.cost_per_kg && a.blade_density == b.blade_density;
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

/// True when the live turbine design differs from the all-time best
/// (i.e. the user edited TurbinePanel sliders between runs).
bool OptimizationSystem::turbine_tweaked() const {
    if (best_design_.size() < kNumDesignVars) return false;
    if (!reg_ || !entity_set(turbine_) || !reg_->valid(turbine_)) return false;
    const auto* spec = reg_->try_get<TurbineSpec>(turbine_);
    if (!spec) return false;
    const float bd[] = {
        static_cast<float>(best_design_[0]), static_cast<float>(best_design_[1]),
        static_cast<float>(best_design_[2]), static_cast<float>(best_design_[3]),
        static_cast<float>(best_design_[4]), static_cast<float>(best_design_[5]),
        static_cast<float>(best_design_[6]), static_cast<float>(best_design_[7]),
    };
    return bd[0] != spec->radius || bd[1] != spec->hub_radius ||
           bd[2] != spec->axial_chord || bd[3] != spec->tip_taper ||
           bd[4] != spec->pitch_deg || bd[5] != spec->twist_deg ||
           bd[6] != spec->thickness || bd[7] != spec->hub_nose;
}

// ── Start / stop ────────────────────────────────────────────────────

void OptimizationSystem::start_optimization() {
    if (running_) return;
    ++run_counter_;

    const OptimizationConfig& cfg = reg_->get<OptimizationConfig>(study_);

    // ── All-time best across runs ─────────────────────────────────
    // The record (best_design_ / current_best_fitness_) intentionally
    // persists across runs so repeated starts can only improve it.
    // Re-baseline only when the environment changed or the user tweaked
    // the turbine design by hand between runs.
    if (!has_baseline_ || !same_config(baseline_config_, cfg) || turbine_tweaked()) {
        has_baseline_ = true;
        baseline_config_ = cfg;
        current_best_fitness_ = std::numeric_limits<double>::infinity();
        best_design_.clear();
        if (reg_ && entity_set(turbine_) && reg_->valid(turbine_)) {
            if (const auto* spec = reg_->try_get<TurbineSpec>(turbine_)) {
                best_design_ = {
                    spec->radius, spec->hub_radius, spec->axial_chord, spec->tip_taper,
                    spec->pitch_deg, spec->twist_deg, spec->thickness, spec->hub_nose
                };
                // Analytic mode can score the baseline immediately; the
                // coupled-CFD mode leaves the record at +inf and lets the
                // first evaluated candidate establish it (each candidate is
                // a real solver run, and the current design is the warm
                // start, so it is evaluated in the first batch anyway).
                if (model_ == ObjectiveModel::Analytic)
                    current_best_fitness_ = evaluate_turbine_design(
                        best_design_, cfg.wind_speed, cfg.viscosity, cfg.air_density,
                        cfg.hub_height, cfg.cost_per_kg, cfg.blade_density);
            }
        }
        completed_run_ = false;
    }

    // Design variables depend on the objective model.
    exd::opt::Problem problem;
    if (model_ == ObjectiveModel::EngineSim) {
        // Steam engine: boiler pressure, cutoff, crank radius, bore.
        problem.variables.push_back({.lower = 2.0e5,  .upper = 2.0e6});  // p_boiler [Pa]
        problem.variables.push_back({.lower = 10.0,   .upper = 120.0});   // cutoff   [deg]
        problem.variables.push_back({.lower = 0.02,   .upper = 0.15});    // crank_radius [m]
        problem.variables.push_back({.lower = 0.05,   .upper = 0.20});    // bore     [m]
    } else {
        // Turbine: 8 design variables (radius, hub, chord, taper, pitch,
        // twist, thickness, hub nose).
        problem.variables.push_back({.lower = 1.0,   .upper = 8.0});    // radius     [m]
    problem.variables.push_back({.lower = 0.2,   .upper = 1.0});    // hub_radius [m]
    problem.variables.push_back({.lower = 0.3,   .upper = 2.5});    // axial_chord [m]
    problem.variables.push_back({.lower = 0.35,  .upper = 1.0});    // tip_taper  [-]
    problem.variables.push_back({.lower = -180.0, .upper = 180.0}); // pitch_deg  [deg]
    problem.variables.push_back({.lower = -90.0,  .upper = 90.0});  // twist_deg  [deg]
    problem.variables.push_back({.lower = 0.06,  .upper = 0.35});   // thickness  [t/c]
    problem.variables.push_back({.lower = 0.0,   .upper = 3.0});    // hub_nose   [m]
    }

    exd::opt::OptimizeOptions opts;
    opts.max_evaluations =
        model_ == ObjectiveModel::CoupledCfd
            ? kCfdMaxEvaluations
            : (model_ == ObjectiveModel::EngineSim ? kEngineMaxEvaluations
                                                   : kMaxEvaluations);
    opts.seed = kSeed + run_counter_;   // fresh trajectory per run, still
                                        // deterministic and reproducible
    opts.n_threads = kNumThreads;
    opts.track_history = true;
    // Continuation runs refine around the current record instead of
    // re-exploring the whole box at full width (CMA-ES default σ = 0.3
    // would wander away from a known good design and usually regress).
    opts.initial_sigma = completed_run_ ? kInitialSigmaRefine : kInitialSigmaFull;

    // Snapshot the environment for coupled-CFD evals so mid-run slider
    // edits cannot corrupt the objective midway through a batch.
    const OptimizationConfig& env = reg_->get<OptimizationConfig>(study_);
    cfd_cfg_ = {env.wind_speed, kCfdRamp, kCfdGrid, kCfdSteps};
    (void)env;
    pending_batch_.clear();
    pending_fitness_.clear();
    pending_done_ = 0;

    // ── Seed from the current design so CMA-ES iterates on it ──
    if (model_ == ObjectiveModel::EngineSim) {
        // fixed fields for the objective come from the CURRENT engine design
        reg_->view<EngineSpec>().each([this](ecs::Entity, const EngineSpec& e) {
            engine_base_ = e;
        });
        const double eng[] = {engine_base_.p_boiler, engine_base_.cutoff_deg,
                              engine_base_.crank_radius, engine_base_.bore};
        opts.initial_mean.resize(problem.dim());
        for (size_t i = 0; i < problem.dim(); ++i) {
            const double lo = problem.variables[i].lower;
            const double hi = problem.variables[i].upper;
            opts.initial_mean[i] = std::clamp((eng[i] - lo) / (hi - lo), 0.0, 1.0);
        }
    } else if (reg_ && entity_set(turbine_) && reg_->valid(turbine_)) {
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
    // NOTE: current_best_fitness_ / best_design_ are NOT reset here —
    // they are the monotonic all-time record for the current environment.

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

    std::printf("[Optimization] CMA-ES %s: v=%.1f m/s, μ=%.1e m²/s, "
                "ρ=%.3f kg/m³, %zu design vars, %zu evals, seed=%zu%s\n",
                completed_run_ ? "refining (warm start)" : "started",
                cfg.wind_speed, cfg.viscosity, cfg.air_density,
                optimizer_->problem().dim(), opts.max_evaluations,
                static_cast<size_t>(kSeed + run_counter_),
                model_ == ObjectiveModel::CoupledCfd ? " (coupled-CFD)" : "");
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

    if (model_ == ObjectiveModel::CoupledCfd) {
        cfd_update(registry);
        return;
    }
    if (model_ == ObjectiveModel::EngineSim) {
        engine_update(registry);
        return;
    }

    // 1. Pull the next batch of candidate designs.
    std::vector<exd::opt::design> batch = optimizer_->request_batch();

    const OptimizationConfig& cfg = registry.get<OptimizationConfig>(study_);

    // 2. An empty batch means the run is finished — finalize.
    if (batch.empty()) {
        running_ = false;
        completed_run_ = true;
        result_ = optimizer_->result();
        // Monotonic guard: adopt the run's verdict only if it actually
        // beats the all-time record (candidates were already checked, so
        // this is defensive — it keeps cross-run progress honest).
        if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
            result_.best_fitness[0] < current_best_fitness_) {
            best_design_ = result_.best_x;
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
    //    engine's final verdict as the authoritative answer (guarded by
    //    the all-time record, see step 2).
    if (!optimizer_->running()) {
        running_ = false;
        completed_run_ = true;
        result_ = optimizer_->result();
        if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
            result_.best_fitness[0] < current_best_fitness_) {
            best_design_ = result_.best_x;
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

// ── Engine-sim objective mode ───────────────────────────────────────

void OptimizationSystem::engine_update(ecs::Registry& registry) {
    // Candidate evaluations are sub-millisecond (0D simulator): evaluate
    // the whole batch inline, mirroring the analytic path.
    std::vector<exd::opt::design> batch = optimizer_->request_batch();
    const EngineSpec base = engine_base_;

    if (batch.empty()) {
        running_ = false;
        completed_run_ = true;
        result_ = optimizer_->result();
        if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
            result_.best_fitness[0] < current_best_fitness_) {
            best_design_ = result_.best_x;
            current_best_fitness_ = result_.best_fitness[0];
        }
        current_generation_ = optimizer_->generation();
        current_evaluations_ = optimizer_->evaluations();
        apply_best_engine_design();
        publish_fitness();
        std::printf("[Optimization] engine-sim finished: %s, %zu evals, "
                    "%zu generations\n",
                    exd::opt::to_string(result_.stop_reason),
                    current_evaluations_, current_generation_);
        return;
    }

    std::vector<exd::opt::Evaluation> evals;
    evals.reserve(batch.size());
    for (const auto& cand : batch) {
        const std::vector<double> eng =
            exd::opt::to_engineering(optimizer_->problem(), cand);
        EngineSpec spec = base;
        apply_engine_params(spec, eng);
        const impl::EngineRunOutcome res = impl::run_engine_eval(spec);
        const double f = res.valid ? -res.mean_power_w : kEnginePenalty;
        if (f < current_best_fitness_) {
            current_best_fitness_ = f;
            best_design_ = eng;
        }
        evals.push_back(exd::opt::Evaluation{{f}});
    }

    optimizer_->submit_results(std::move(evals));
    if (!optimizer_->running()) {
        running_ = false;
        completed_run_ = true;
        result_ = optimizer_->result();
        if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
            result_.best_fitness[0] < current_best_fitness_) {
            best_design_ = result_.best_x;
            current_best_fitness_ = result_.best_fitness[0];
        }
    }
    current_generation_ = optimizer_->generation();
    current_evaluations_ = optimizer_->evaluations();
    apply_best_engine_design();
    publish_fitness();
}

void OptimizationSystem::apply_best_engine_design() {
    if (!reg_ || best_design_.size() < 4) return;
    // The engine is located by its component; write EngineSpec and
    // SteamEngineSystem picks it up on its next update.
    reg_->view<EngineSpec>().each([this](ecs::Entity, EngineSpec& e) {
        apply_engine_params(e, best_design_);
    });
}

// ── Coupled-CFD objective mode ──────────────────────────────────────

void OptimizationSystem::cfd_update(ecs::Registry& registry) {
    // 1. Poll the worker; adopt the finished candidate evaluation.
    if (cfd_busy_.load(std::memory_order_relaxed)) {
        if (cfd_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            return;
        std::unique_ptr<CfdEvalResult> res;
        try {
            res = cfd_future_.get();
        } catch (...) {
            res = std::make_unique<CfdEvalResult>();
            res->error = "worker exception";
        }
        cfd_busy_.store(false, std::memory_order_relaxed);
        if (cfd_run_token_ != run_counter_) return;   // stale run: discard

        const double f = res ? res->fitness : kCfdPenalty;
        if (pending_done_ < pending_fitness_.size())
            pending_fitness_[pending_done_] = f;
        if (res && res->valid && f < current_best_fitness_) {
            current_best_fitness_ = f;
            best_design_ = exd::opt::to_engineering(
                optimizer_->problem(), pending_batch_[pending_done_]);
            apply_best_design();
        }
        std::printf("[Optimization] candidate %zu/%zu: Cp=%.3f (%.2fs)\n",
                    pending_done_ + 1, pending_batch_.size(),
                    res ? res->cp : 0.0, res ? res->wall_seconds : 0.0);
        ++pending_done_;

        // Batch complete → hand all λ results to the optimizer.
        if (pending_done_ == pending_batch_.size()) {
            std::vector<exd::opt::Evaluation> evals;
            evals.reserve(pending_fitness_.size());
            for (double f : pending_fitness_)
                evals.push_back(exd::opt::Evaluation{{f}});
            pending_batch_.clear();
            pending_fitness_.clear();
            pending_done_ = 0;
            optimizer_->submit_results(std::move(evals));
            if (!optimizer_->running()) {
                running_ = false;
                completed_run_ = true;
                result_ = optimizer_->result();
                if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
                    result_.best_fitness[0] < current_best_fitness_) {
                    best_design_ = result_.best_x;
                    current_best_fitness_ = result_.best_fitness[0];
                }
            }
            current_generation_ = optimizer_->generation();
            current_evaluations_ = optimizer_->evaluations();
            apply_best_design();
            publish_fitness();
            return;
        }
    }

    // 2. Refill the queue when idle.
    if (cfd_busy_.load(std::memory_order_relaxed)) return;
    if (pending_batch_.empty()) {
        const std::vector<exd::opt::design> batch = optimizer_->request_batch();
        if (batch.empty()) {
            // Run finished (budget/convergence): finalize like the
            // analytic path.
            running_ = false;
            completed_run_ = true;
            result_ = optimizer_->result();
            if (!result_.best_x.empty() && !result_.best_fitness.empty() &&
                result_.best_fitness[0] < current_best_fitness_) {
                best_design_ = result_.best_x;
                current_best_fitness_ = result_.best_fitness[0];
            }
            current_generation_ = optimizer_->generation();
            current_evaluations_ = optimizer_->evaluations();
            apply_best_design();
            publish_fitness();
            std::printf("[Optimization] coupled-CFD finished: %s, %zu evals, "
                        "%zu generations\n",
                        exd::opt::to_string(result_.stop_reason),
                        current_evaluations_, current_generation_);
            return;
        }
        pending_batch_ = batch;
        pending_fitness_.assign(batch.size(), kCfdPenalty);
        pending_done_ = 0;
    }

    // 3. Launch the next candidate (one worker at a time).
    if (pending_done_ < pending_batch_.size()) {
        TurbineSpec design;
        apply_params(design,
                     exd::opt::to_engineering(optimizer_->problem(),
                                              pending_batch_[pending_done_]));
        const CfdEvalCfg cfg = cfd_cfg_;
        cfd_run_token_ = run_counter_;
        cfd_future_ = std::async(std::launch::async, cfd_eval_worker, design,
                                 cfg.wind_speed, cfg.ramp_time_s, cfg.grid,
                                 cfg.steps);
        cfd_busy_.store(true, std::memory_order_relaxed);
        std::printf("[Optimization] evaluating candidate %zu/%zu (gen %zu)\n",
                    pending_done_ + 1, pending_batch_.size(),
                    current_generation_ + 1);
    }
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
    rec.evals_pending = (running_ && model_ == ObjectiveModel::CoupledCfd &&
                         !pending_batch_.empty())
                            ? static_cast<int>(pending_batch_.size() - pending_done_)
                            : 0;
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

    ImGui::Text(model_ == ObjectiveModel::EngineSim
                    ? "CMA-ES steam engine optimization"
                    : "CMA-ES turbine design optimization");
    ImGui::Separator();

    if (model_ != ObjectiveModel::EngineSim) {
        // ── Environment settings (stored in OptimizationConfig) ──
        ImGui::Text("Environment");
        ImGui::SliderFloat("Wind speed [m/s]", &cfg.wind_speed, 1.0f, 40.0f, "%.1f");
        ImGui::SliderFloat("Viscosity [m²/s]", &cfg.viscosity, 5.0e-6f, 5.0e-4f, "%.1e");
        ImGui::SliderFloat("Air density [kg/m³]", &cfg.air_density, 0.5f, 1.5f, "%.3f");
        ImGui::SliderFloat("Hub height [m]", &cfg.hub_height, 5.0f, 100.0f, "%.0f");
        ImGui::Separator();
        ImGui::SliderFloat("Cost per kg", &cfg.cost_per_kg, 0.1f, 10.0f, "%.1f");
        ImGui::SliderFloat("Blade density [kg/m³]", &cfg.blade_density, 500.0f, 3000.0f, "%.0f");
        ImGui::Separator();
    }

    // ── Start / stop ──
    if (running_) {
        if (ImGui::Button("Stop Optimization")) stop_optimization();
    } else {
        if (ImGui::Button("Start Optimization")) start_optimization();
    }

    ImGui::Separator();
    ImGui::Text("Status: %s", status_text());
    ImGui::Text("Generation: %d", rec.generation);
    if (model_ == ObjectiveModel::CoupledCfd) {
        ImGui::Text("Evaluations: %d / %zu", rec.evaluations, kCfdMaxEvaluations);
        if (rec.running && rec.evals_pending > 0)
            ImGui::Text("Evaluating candidate %d of batch...", rec.evals_pending);
    } else {
        ImGui::Text("Evaluations: %d / %zu", rec.evaluations, kMaxEvaluations);
    }
    if (std::isfinite(rec.best_fitness)) {
        ImGui::Text("Best fitness (cumulative): %.4f", rec.best_fitness);
    } else {
        ImGui::Text("Best fitness (cumulative): --");
    }

    ImGui::Separator();
    ImGui::Text("Best design so far");
    if (rec.evaluations > 0 && model_ == ObjectiveModel::EngineSim) {
        ImGui::Text("Boiler:       %.2f MPa",   rec.best_design[0] * 1e-6);
        ImGui::Text("Cutoff:       %.1f deg",   rec.best_design[1]);
        ImGui::Text("Crank radius: %.3f m",     rec.best_design[2]);
        ImGui::Text("Bore:         %.3f m",     rec.best_design[3]);
    } else if (rec.evaluations > 0) {
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
    if (model_ == ObjectiveModel::EngineSim) {
        ImGui::TextWrapped(
            "Objective: maximize mean indicated power of the 0D steam "
            "engine (Rankine-lite: admission to cutoff, wet-steam "
            "polytrope, exhaust to condenser). Variables: boiler "
            "pressure, cutoff, crank radius, bore. The best design is "
            "written to the Steam Engine panel's spec and persists "
            "across runs (monotonic record).");
    } else {
        ImGui::TextWrapped(
            "Objective: maximize (power revenue − blade cost). "
            "Power uses actuator-disk model with Schmitz Cp, "
            "Reynolds-number efficiency, and drivetrain losses. "
            "Blade mass scales with R × c × t × density. "
            "The best design persists across runs for the current "
            "environment: each restart refines it with a fresh search, "
            "so repeated runs keep improving the record (it never "
            "regresses). Changing the environment resets the record. "
            "One generation per frame.");
    }
}

} // namespace exd::sim
