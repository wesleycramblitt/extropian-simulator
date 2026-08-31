#pragma once
// ─────────────────────────────────────────────────────────────────────
// SolverRun components — one-shot coupled-FDM3 turbine run on demand.
//
// The "SolverRun" entity is created by SolverRunSystem. The panel edits
// SolverRunConfig; the system runs a full coupled turbine-in-grid CFD
// simulation in the background (run_coupled_turbine on a worker thread)
// and publishes the outcome in SolveRunState. The current TurbineSpec on
// the turbine entity defines the rotor geometry the run solves.
//
// Both structs are POD and satisfy the exd::ecs::Component concept.
// ─────────────────────────────────────────────────────────────────────

namespace exd::sim {

/// Panel-editable configuration for the coupled FDM3 turbine run.
/// Writers: SolverRunSystem's panel.  Readers: SolverRunSystem.
struct SolverRunConfig {
    float wind_speed        = 8.0f;   // m/s  inflow (box sized from current radius)
    int   n_per_axis        = 18;     // grid cells per axis (18³ ≈ 5.8k cells, ~1-2 s)
    int   max_steps         = 4000;   // fluid step budget (exchange every 12 steps)
    float ramp_time_s       = 1.0f;   // forcing ramp; must be ≥ 10·exchange window
    float radius_margin     = 1.8f;   // box half-width in rotor radii (lateral)
    float wake_length_radii = 3.0f;   // box half-length upstream/downstream in radii
};

/// Live status of the most recent coupled run (written each frame).
/// Writers: SolverRunSystem.  Readers: panels, dashboards.
struct SolveRunState {
    enum Status : uint8_t { Idle = 0, Running = 1, Done = 2, Failed = 3 };

    uint8_t status = Idle;
    int  grid_nx = 0, grid_ny = 0, grid_nz = 0;   // solved grid dims
    int  steps_taken = 0;                          // fluid steps executed
    double final_cp   = 0.0;                       // power coefficient
    double final_tsr  = 0.0;                       // tip-speed ratio
    double final_power_kw = 0.0;                   // shaft power (kW)
    double wall_seconds = 0.0;                     // wall time of the run
    // Decimated shaft-power trace over run time (POD, for dashboards).
    static constexpr int kTraceSamples = 64;
    int   trace_count = 0;
    float power_kw_trace[kTraceSamples] = {};
};

} // namespace exd::sim
