// ─────────────────────────────────────────────────────────────────────
// Headless test for the engine-run recipe (src/engine_run.hpp), shared by
// SteamEngineSystem and OptimizationSystem (engine-sim mode).
//
// Asserts:
//   1. the steam config validates and produces positive mean power,
//   2. two identical runs are bit-for-bit deterministic,
//   3. sensitivity: higher boiler pressure raises mean power,
//   4. degenerate designs are rejected cleanly.
// ─────────────────────────────────────────────────────────────────────
#include "engine_run.hpp"

#include <cstdio>

int main() {
    std::printf("Engine-run recipe test: 0D steam engine...\n");

    exd::sim::EngineSpec spec;   // canonical defaults
    const auto run = [&](exd::sim::EngineSpec s) {
        return exd::sim::impl::run_engine_eval(s);
    };

    const auto r1 = run(spec);
    if (!r1.valid) {
        std::printf("FAIL: run invalid: %s\n", r1.error.c_str());
        return 1;
    }
    if (r1.mean_power_w <= 0.0) {
        std::printf("FAIL: no positive power (%.2f W)\n", r1.mean_power_w);
        return 1;
    }

    const auto r2 = run(spec);
    if (r1.mean_power_w != r2.mean_power_w || r1.cycles != r2.cycles) {
        std::printf("FAIL: non-deterministic: P %.4f vs %.4f\n",
                    r1.mean_power_w, r2.mean_power_w);
        return 1;
    }
    std::printf("OK: valid + deterministic: P=%.1f W, ω=%.1f rad/s, "
                "%.0f cycles, η=%.3f (%.2f s wall, %zu indicator samples)\n",
                r1.mean_power_w, r1.mean_omega_rad_s, r1.cycles,
                r1.efficiency, r1.wall_seconds, r1.indicator.size());

    // Sensitivity: double the boiler pressure → more power expected.
    auto hot = spec;
    hot.p_boiler = spec.p_boiler * 1.6f;
    const auto rh = run(hot);
    if (!rh.valid || rh.mean_power_w <= r1.mean_power_w) {
        std::printf("FAIL: boiler-pressure sensitivity broken: "
                    "P(1.0x)=%.1f vs P(1.6x)=%.1f\n",
                    r1.mean_power_w, rh.valid ? rh.mean_power_w : -1.0);
        return 1;
    }
    std::printf("OK: sensitivity: boiler 1.6x → P=%.1f W (was %.1f)\n",
                rh.mean_power_w, r1.mean_power_w);

    // Degenerate: zero crank radius rejected.
    auto bad = spec;
    bad.crank_radius = 0.0f;
    const auto rb = run(bad);
    if (rb.valid) {
        std::printf("FAIL: degenerate design accepted\n");
        return 1;
    }
    std::printf("OK: degenerate design rejected (%s)\n", rb.error.c_str());
    return 0;
}
