#pragma once

#include <limits>
// ─────────────────────────────────────────────────────────────────────
// Optimization components — the study entity and design results.
//
// The optimization "study" is an ECS entity owned by OptimizationSystem.
// The panel edits OptimizationConfig; the system publishes FitnessRecord
// as results accumulate. OptimizationSystem drives the turbine design by
// writing the TurbineSpec component on the turbine entity (see
// components/turbine.hpp) — never through direct system references.
//
// Both structs are POD and satisfy the exd::ecs::Component concept
// (std::numeric_limits keeps it trivially constructible/destructible).
// ─────────────────────────────────────────────────────────────────────

namespace exd::sim {

/// Live-editable environment + cost settings used by the objective.
/// Writers: OptimizationSystem's panel.  Readers: OptimizationSystem.
struct OptimizationConfig {
    float wind_speed    = 10.0f;   // m/s  (typical onshore 5–25)
    float viscosity     = 1.5e-5f; // m²/s (air at ~15°C)
    float air_density   = 1.225f;  // kg/m³ (sea-level standard)
    float hub_height    = 30.0f;   // m   (wind shear reference)
    float cost_per_kg   = 1.0f;    // relative cost factor
    float blade_density = 1800.0f; // kg/m³ (fiberglass/epoxy composite)
};

/// Number of design variables in the optimization study (fixed vector
/// length so FitnessRecord stays POD).
inline constexpr int kOptimizationDesignVars = 8;

/// Published results of the CMA-ES study, written to the study entity.
/// best_fitness / best_design are the ALL-TIME best across runs for the
/// current environment configuration (monotonic — they never regress);
/// generation / evaluations describe the most recent run.
/// Writers: OptimizationSystem.  Readers: panels, post-processing systems.
struct FitnessRecord {
    double best_fitness = std::numeric_limits<double>::infinity(); // (lower = better)
    float  best_design[kOptimizationDesignVars] = {0.0f};
    int    generation   = 0;
    int    evaluations  = 0;
    int    evals_pending = 0;      // candidates still queued in the current batch
    bool   running      = false;
};

} // namespace exd::sim
