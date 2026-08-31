#pragma once
// ─────────────────────────────────────────────────────────────────────
// Internal shared mapping: ECS EngineSpec → 0D steam engine simulation.
// Single source of truth for SteamEngineSystem, OptimizationSystem's
// engine-objective mode and engine_run_test. NOT installed (src/ include).
// ─────────────────────────────────────────────────────────────────────
#include <exd/physics/engine/engine_config.hpp>
#include <exd/physics/engine/engine_result.hpp>
#include <exd/physics/engine/engine_simulator.hpp>
#include <exd/physics/model_status.hpp>
#include <exd/sim/components/engine.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace exd::sim::impl {

/// Map the ECS engine design into the physics EngineConfig (steam cycle).
inline exd::physics::engine::EngineConfig make_engine_config(
    const EngineSpec& e, double initial_omega = 50.0,
    uint64_t max_steps = 20000) {
    using namespace exd::physics;
    engine::EngineConfig cfg;

    cfg.geometry.crank_radius     = e.crank_radius;
    cfg.geometry.rod_length       = e.rod_length;
    cfg.geometry.bore             = e.bore;
    cfg.geometry.clearance_volume = e.clearance_volume;
    cfg.geometry.piston_mass      = e.piston_mass;
    cfg.geometry.flywheel_inertia = e.flywheel_inertia;

    cfg.thermo.cycle            = engine::EngineCycleType::Steam;
    cfg.thermo.r_gas            = 461.5;             // steam
    cfg.thermo.p_intake         = e.p_boiler;
    cfg.thermo.p_exhaust        = e.p_condenser;
    cfg.thermo.p_back           = e.p_condenser;
    cfg.thermo.p_boiler         = e.p_boiler;
    cfg.thermo.p_condenser      = e.p_condenser;
    cfg.thermo.steam_cutoff_deg = e.cutoff_deg;
    cfg.thermo.steam_gamma      = 1.13;              // wet-steam polytrope
    cfg.thermo.steam_quality_cutoff = e.steam_quality;

    cfg.load.friction_constant  = e.friction_constant;
    cfg.load.friction_viscous   = e.friction_viscous;
    // Generator load so the engine settles at a realistic speed instead of
    // running away (Q ≈ 0 near rest, ~10 N·m at 300 rad/s ≈ 3 kW class).
    cfg.load.generator_enabled = true;
    cfg.load.generator_omega_pts = {0.0, 50.0, 150.0, 300.0, 600.0};
    cfg.load.generator_torque_pts = {0.0, 2.0, 6.0, 10.0, 12.0};

    cfg.dt = 5.0e-4;
    cfg.max_steps = max_steps;
    cfg.initial_omega = initial_omega;   // crank won't self-start from rest
    cfg.record_history = true;
    cfg.history_interval = 10;           // keep the result compact
    cfg.csv_path.clear();
    return cfg;
}

/// Outcome of a completed engine run (main-thread adoptable).
struct EngineRunOutcome {
    bool valid = false;
    std::string error;
    double total_time_s = 0.0;
    double mean_power_w = 0.0;
    double mean_omega_rad_s = 0.0;
    double cycles = 0.0;
    double efficiency = 0.0;
    double wall_seconds = 0.0;
    // indicator samples (θdeg, p_kPa, V_litres) at the stored history cadence
    struct Sample { float theta_deg = 0, p_kpa = 0, v_litres = 0; };
    std::vector<Sample> indicator;
};

/// Run the steam engine simulator to completion. Worker-safe.
inline EngineRunOutcome run_engine_eval(const EngineSpec& e) {
    using namespace std::chrono;
    EngineRunOutcome outcome;
    const double t0 = duration<double>(
        steady_clock::now().time_since_epoch()).count();

    exd::physics::ModelStatus status;
    const auto cfg = make_engine_config(e);
    const exd::physics::engine::EngineSimResult res =
        exd::physics::engine::simulate_engine(cfg, status);
    outcome.wall_seconds = duration<double>(
        steady_clock::now().time_since_epoch()).count() - t0;

    if (!status.ok || !res.valid) {
        outcome.error = status.ok ? res.error : status.error;
        return outcome;
    }

    outcome.valid = true;
    outcome.total_time_s = res.total_time;
    outcome.mean_power_w = res.mean_indicated_power;
    outcome.mean_omega_rad_s = res.mean_omega;
    outcome.cycles = res.cycles_completed;
    outcome.efficiency = res.efficiency_estimate;
    outcome.indicator.reserve(res.history.size());
    for (const auto& h : res.history) {
        outcome.indicator.push_back({
            static_cast<float>(h.state.theta_rad * 180.0 / 3.14159265358979323846),
            static_cast<float>(h.p_cyl * 1e-3),
            static_cast<float>(h.V_cyl * 1e3),
        });
    }
    return outcome;
}

} // namespace exd::sim::impl
