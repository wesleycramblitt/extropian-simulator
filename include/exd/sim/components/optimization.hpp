#pragma once

#include <cstdint>
#include <limits>
// ─────────────────────────────────────────────────────────────────────
// Optimization components — the study entity and design results.
//
// The optimization "study" is an ECS entity owned by OptimizationSystem.
// The panel edits OptimizationConfig; the system publishes FitnessRecord
// as results accumulate.
//
// OptimizationSystem is GENERIC: it optimizes a design vector over
// engineering-space bounds using a caller-provided objective (injected at
// construction — see optimization_system.hpp). Nothing in these
// components references a specific domain (turbine, engine, ...); the
// domain that owns the variables maps FitnessRecord back into its own
// components when it wants to apply the found design.
//
// Both structs are POD and satisfy the exd::ecs::Component concept.
// ─────────────────────────────────────────────────────────────────────

namespace exd::sim {

/// Number of design variables in the optimization study (fixed vector
/// length so FitnessRecord stays POD).
inline constexpr int kOptimizationDesignVars = 8;

/// Panel-editable study definition: variables, algorithm, budget.
/// Writers: OptimizationSystem's panel.  Readers: OptimizationSystem.
struct OptimizationConfig {
    /// Engineering-space bounds per variable (only the first `n_vars`
    /// entries are active).
    float lower[kOptimizationDesignVars] = {0.0f};
    float upper[kOptimizationDesignVars] = {1.0f};
    int   n_vars = 2;                 ///< active dims, 1..kOptimizationDesignVars
    int   algo = 0;                   ///< exd::opt::Algo as int (0 = CMA-ES)
    int   max_evaluations = 2000;     ///< per-run budget
    std::uint64_t seed = 42;          ///< base seed; each run gets seed + run#
};

/// Published results of the study, written to the study entity. Best
/// values describe the MOST RECENT run (best_fitness of +inf = no
/// evaluated candidate yet). Values are in engineering units.
/// Writers: OptimizationSystem.  Readers: panels, domain systems.
struct FitnessRecord {
    double best_fitness = std::numeric_limits<double>::infinity(); // lower = better
    float  best_design[kOptimizationDesignVars] = {0.0f};
    int    generation   = 0;
    int    evaluations  = 0;
    int    evals_pending = 0;      // candidates still queued in the current batch
    bool   running      = false;
};

} // namespace exd::sim
