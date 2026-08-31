#pragma once
// ─────────────────────────────────────────────────────────────────────
// Internal shared mapping: ECS turbine design → coupled FDM3 run.
// Single source of truth for SolverRunSystem and the coupled-CFD
// objective of OptimizationSystem (and solver_run_test, which asserts
// validity + determinism on this exact recipe).
// NOT installed; consumers include it directly (src/ include path).
//
// Recipe summary (measured, see solver_run_test):
//   • twist hub = 0, tip = −pitch (physics-convention sign flip; the
//     visual twist ramp is aesthetic and intentionally not solved)
//   • box spans [0, lx]×[0, ly]×[0, lz]; rotor at (1.8R, 3+R, 0.45·lz)
//   • dt recomputed after the box override (CFL-clamped)
//   • ramp floored at 10·exchange-window for the driver's validation
//   • generator load curve sized to the demo machine (explicit points,
//     NOT make_generator_curve(P, η, min_ω) — that formula blows up
//     when min_ω « P/η and pins the rotor before spin-up)
// ─────────────────────────────────────────────────────────────────────
#include <exd/physics/fluid/fdm3/fdm3_result.hpp>
#include <exd/physics/model_status.hpp>
#include <exd/physics/turbine/coupled_turbine.hpp>
#include <exd/physics/turbine/turbine_builder.hpp>
#include <exd/sim/components/turbine.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace exd::sim::impl {

/// Build the coupled-driver configuration from an ECS design + run params.
inline exd::physics::turbine::CoupledTurbineConfig make_coupled_run_config(
    const TurbineSpec& spec, double wind_speed, int n_per_axis, int max_steps,
    double ramp_time_s, double radius_margin, double wake_length_radii,
    exd::physics::ModelStatus& status) {
    using namespace exd::physics;
    using turbine::CoupledTurbineConfig;

    turbine::TurbineBuilderConfig b;
    b.hub_radius       = spec.hub_radius;
    b.tip_radius       = spec.radius;
    b.chord            = spec.axial_chord;
    b.twist_hub_deg    = 0.0;                       // flat root (model fidelity)
    b.twist_tip_deg    = -static_cast<double>(spec.pitch_deg);
    b.rpm              = spec.rpm;
    b.blade_count      = spec.blade_count;
    b.section_count    = 5;
    const double chord = std::max(static_cast<double>(spec.axial_chord), 0.3);
    b.leading_edge_z   = 0.5 * chord;
    b.duct_length      = b.leading_edge_z + chord + 1.0;
    b.shroud_radius    = 0.0;
    b.default_airfoil  = "naca0012";
    const geometry::TurbineDefinition def = turbine::make_turbine_definition(b, status);
    if (!status.ok) return {};

    CoupledTurbineConfig cc;
    const double R = std::max(static_cast<double>(spec.radius), 0.5);
    const double hub_y = 3.0 + static_cast<double>(spec.radius);
    cc.grid = turbine::default_grid_config(wind_speed, n_per_axis, 1.0, 1.0);
    cc.grid.lx = 2.0 * R * radius_margin;
    cc.grid.ly = 2.0 * (hub_y + R * radius_margin);
    cc.grid.lz = 2.0 * R * wake_length_radii;
    cc.grid.dt = 0.25 * std::min({cc.grid.dx(), cc.grid.dy(), cc.grid.dz()})
                 / std::max(wind_speed, 1e-9);
    cc.turbine = def;
    cc.element_count = 8;
    cc.rotor_inertia = 1200.0;
    mechanics::CurveMomentConfig generator;
    generator.omega_pts  = {0.0, 2.0, 8.0, 12.0, 30.0};
    generator.torque_pts = {0.0, 40.0, 368.0, 245.0, 98.0};
    cc.generator = generator;
    cc.fluid_steps_per_exchange = 12;
    cc.force_relaxation = 0.4;
    cc.smear_cells = 2.5;
    cc.max_steps = static_cast<uint64_t>(std::max(max_steps, 100));
    cc.record_history = true;
    cc.history_interval = 12;
    cc.csv_path.clear();

    const double m = R * radius_margin;
    const double u = 0.45 * cc.grid.lz;
    cc.rotor_origin = {m, hub_y, u};

    const double dt = cc.grid.dt;
    const double window_s = static_cast<double>(cc.fluid_steps_per_exchange) * dt;
    cc.ramp_time_s = std::max(ramp_time_s, 10.0 * window_s + 1e-9);
    return cc;
}

/// Outcome of a completed coupled run (main-thread adoptable; no registry).
struct CoupledRunOutcome {
    bool valid = false;
    std::string error;
    int nx = 0, ny = 0, nz = 0, steps_taken = 0;
    double final_cp = 0.0, final_tsr = 0.0, power_w = 0.0, wall_seconds = 0.0;
    std::vector<double> power_w_trace;   // one entry per exchange
};

/// Run the coupled CFD to completion. Worker-safe: touches no registry,
/// no render, no ImGui. When `out_field` is non-null the final field is
/// moved into it (skip for objective evals — big allocation).
inline CoupledRunOutcome run_coupled_eval(const TurbineSpec& spec, double wind_speed,
                                   int n_per_axis, int max_steps,
                                   double ramp_time_s, double radius_margin,
                                   double wake_length_radii,
                                   exd::physics::fluid::fdm3::FDM3FieldData* out_field) {
    using namespace std::chrono;
    CoupledRunOutcome outcome;
    const double t0 = duration<double>(
        steady_clock::now().time_since_epoch()).count();

    exd::physics::ModelStatus status;
    auto cc = make_coupled_run_config(spec, wind_speed, n_per_axis, max_steps,
                                      ramp_time_s, radius_margin,
                                      wake_length_radii, status);
    if (!status.ok) {
        outcome.error = status.error.empty() ? "invalid run configuration" : status.error;
        return outcome;
    }

    const exd::physics::turbine::CoupledTurbineResult res =
        exd::physics::turbine::run_coupled_turbine(cc, status);
    outcome.wall_seconds = duration<double>(
        steady_clock::now().time_since_epoch()).count() - t0;

    if (!status.ok || !res.valid) {
        outcome.error = status.ok ? res.error : status.error;
        return outcome;
    }

    outcome.valid = true;
    outcome.nx = res.fluid.field.nx;
    outcome.ny = res.fluid.field.ny;
    outcome.nz = res.fluid.field.nz;
    outcome.steps_taken = res.fluid.steps_taken;
    outcome.final_cp = res.final_cp;
    outcome.final_tsr = res.final_tsr;
    outcome.power_w = res.history.empty() ? 0.0 : res.history.back().power;
    outcome.power_w_trace.reserve(res.history.size());
    for (const auto& h : res.history) outcome.power_w_trace.push_back(h.power);
    if (out_field) *out_field = std::move(res.fluid.field);
    return outcome;
}

} // namespace exd::sim::impl
