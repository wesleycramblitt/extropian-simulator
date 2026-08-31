#pragma once
// ─────────────────────────────────────────────────────────────────────
// Steam engine components — the machine design, run state and indicator.
//
// The "SteamEngine" entity is owned by SteamEngineSystem. The panel edits
// EngineSpec (geometry + steam cycle); the system runs the 0D Rankine-lite
// slider-crank simulator (exd::physics::engine) in the background and
// publishes EngineRunState (stats) and IndicatorRecord (pressure / volume
// samples for the indicator diagram, fixed-size so it stays a POD
// component). OptimizationSystem (engine mode) also writes EngineSpec.
//
// All structs are POD and satisfy the exd::ecs::Component concept.
// ─────────────────────────────────────────────────────────────────────

namespace exd::sim {

/// Panel-editable steam engine design.
/// Writers: SteamEngineSystem panel, OptimizationSystem (engine mode).
/// Readers: SteamEngineSystem.
struct EngineSpec {
    // ── Mechanism geometry (same units as exd::physics::engine) ──
    float crank_radius   = 0.05f;    // m  (> 0)
    float rod_length     = 0.20f;    // m  (> crank_radius)
    float bore           = 0.086f;   // m  cylinder diameter (> 0)
    float clearance_volume = 1.0e-4f; // m³ TDC volume
    float piston_mass    = 0.5f;     // kg
    float flywheel_inertia = 0.02f;  // kg·m²

    // ── Steam cycle ──
    float p_boiler       = 800000.0f;  // Pa admission (must exceed condenser)
    float p_condenser    = 15000.0f;   // Pa exhaust
    float cutoff_deg     = 40.0f;      // ° admission cutoff, (0, 180)
    float steam_quality  = 0.95f;      // dryness at cutoff, (0, 1]

    // ── Load ──
    float friction_constant = 0.3f;    // N·m
    float friction_viscous  = 0.10f;   // N·m·s/rad (caps runaway; the 0D model
                                       // has constant per-rev work, so a
                                       // speed-proportional load sets ω_eq)
};

/// Live status of the most recent engine simulation.
/// Writers: SteamEngineSystem.  Readers: panels, dashboards.
struct EngineRunState {
    enum Status : uint8_t { Idle = 0, Running = 1, Done = 2, Failed = 3 };

    uint8_t status = Idle;
    int  steps_taken = 0;
    double total_time_s = 0.0;
    double cycles = 0.0;
    double mean_power_w = 0.0;
    double mean_omega_rad_s = 0.0;
    double efficiency = 0.0;
    double wall_seconds = 0.0;
};

/// Fixed-size indicator-diagram samples (pressure / volume per crank angle),
/// rebuilt after every completed engine run. POD so dashboards can read it
/// through the registry.
/// Writers: SteamEngineSystem.  Readers: panels, spatial-ui dashboards.
struct IndicatorRecord {
    static constexpr int kMaxSamples = 256;
    int   count = 0;
    float crank_deg[kMaxSamples] = {};
    float pressure_kpa[kMaxSamples] = {};
    float volume_litres[kMaxSamples] = {};
};

} // namespace exd::sim
