// ─────────────────────────────────────────────────────────────────────
// Headless test for the coupled-run recipe (src/coupled_run.hpp), the
// single source of truth used by SolverRunSystem and OptimizationSystem.
//
// Asserts:
//   1. the mapping produces a VALID coupled-run configuration (the ramp /
//      containment / stability validation in extropian-physics passes),
//   2. the rotor spins up the correct way (TSR > 0),
//   3. two identical runs are bit-for-bit deterministic (same final Cp),
//   4. a run without a field copy stays valid (objective-eval code path).
// ─────────────────────────────────────────────────────────────────────
#include "coupled_run.hpp"

#include <exd/engine/physics/fluid/fdm3/fdm3_result.hpp>

#include <cstdio>

int main() {
    std::printf("Coupled-run recipe test: tiny FDM3 run, determinism...\n");

    exd::sim::TurbineSpec spec;                       // canonical demo defaults
    const double wind = 8.0;
    const int n = 12, steps = 1500;

    // 1+2+3: full run with field
    {
        exd::engine::physics::fluid::fdm3::FDM3FieldData field;
        auto r1 = exd::sim::impl::run_coupled_eval(spec, wind, n, steps, 1.0, 1.8, 3.0, &field);
        if (!r1.valid) {
            std::printf("FAIL: run invalid: %s\n", r1.error.c_str());
            return 1;
        }
        if (field.u.empty() || field.nx != n) {
            std::printf("FAIL: field not returned\n");
            return 1;
        }
        if (r1.final_tsr <= 0.0) {
            std::printf("FAIL: rotor did not spin up (TSR=%.3f)\n", r1.final_tsr);
            return 1;
        }
        auto r2 = exd::sim::impl::run_coupled_eval(spec, wind, n, steps, 1.0, 1.8, 3.0, nullptr);
        if (r1.final_cp != r2.final_cp || r1.final_tsr != r2.final_tsr) {
            std::printf("FAIL: non-deterministic runs: Cp %.6f vs %.6f\n",
                        r1.final_cp, r2.final_cp);
            return 1;
        }
        std::printf("OK: valid + deterministic: Cp=%.4f TSR=%.2f P=%.1f W "
                    "(%dx%dx%d grid, %.1f s wall, field %zu cells)\n",
                    r1.final_cp, r1.final_tsr,
                    r1.power_w, r1.nx, r1.ny, r1.nz, r1.wall_seconds, field.u.size());
    }

    // 4: objective-eval path (no field copy)
    {
        auto r = exd::sim::impl::run_coupled_eval(spec, wind, n, steps, 1.0, 1.8, 3.0, nullptr);
        if (!r.valid) {
            std::printf("FAIL: eval-path run invalid: %s\n", r.error.c_str());
            return 1;
        }
        std::printf("OK: no-field eval path valid (Cp=%.4f)\n", r.final_cp);
    }

    // Configuration invalidity propagates cleanly (zero radius).
    {
        auto bad = spec;
        bad.radius = 0.0f;
        auto r = exd::sim::impl::run_coupled_eval(bad, wind, n, steps, 1.0, 1.8, 3.0, nullptr);
        if (r.valid) {
            std::printf("FAIL: degenerate design accepted\n");
            return 1;
        }
        std::printf("OK: degenerate design rejected (%s)\n", r.error.c_str());
    }
    return 0;
}
