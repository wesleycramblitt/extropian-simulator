// ─────────────────────────────────────────────────────────────────────
// Headless test for the generic OptimizationSystem. Drives the embedded
// frame loop (one batch per update) against a Rosenbrock objective over
// [-5, 5]^2 and asserts the system converges near the known optimum
// (a, b) = (1, 1) and publishes the results through FitnessRecord.
// ─────────────────────────────────────────────────────────────────────
#include <exd/sim/optimization_system.hpp>

#include <exd/ecs/registry.hpp>
#include <exd/ecs/view.hpp>

#include <cmath>
#include <cstdio>

int main() {
    std::printf("Generic OptimizationSystem test: Rosenbrock frame loop...\n");

    // Objective in ENGINEERING units (the system maps candidates for us).
    exd::sim::OptimizationSystem sys(
        [](const exd::opt::design& x) -> exd::opt::Evaluation {
            const double a = x.size() > 0 ? x[0] : 0.0;
            const double b = x.size() > 1 ? x[1] : 0.0;
            const double f = (1.0 - a) * (1.0 - a)
                           + 100.0 * (b - a * a) * (b - a * a);
            return {.fitness = {f}};
        });

    exd::ecs::Registry reg;

    // 1. Bind the registry + create the study entity.
    sys.update(reg, 1.0 / 60.0);

    // 2. Configure the study: Nelder-Mead over [-5,5]^2, tight budget.
    exd::ecs::Entity study = {};
    reg.view<exd::sim::OptimizationConfig>().each(
        [&](exd::ecs::Entity e, exd::sim::OptimizationConfig& cfg) {
            study = e;
            cfg.n_vars = 2;
            cfg.lower[0] = -5.0f; cfg.upper[0] = 5.0f;
            cfg.lower[1] = -5.0f; cfg.upper[1] = 5.0f;
            cfg.algo = static_cast<int>(exd::opt::Algo::NelderMead);
            cfg.max_evaluations = 500;
        });
    if (!reg.valid(study)) {
        std::printf("FAIL: study entity not created\n");
        return 1;
    }

    // 3. Run the frame loop to completion.
    sys.start_optimization();
    int frames = 0;
    while (sys.is_running() && frames < 10000) {
        sys.update(reg, 1.0 / 60.0);
        ++frames;
    }
    sys.update(reg, 1.0 / 60.0);   // final frame: adopt the verdict

    if (!sys.result().ok()) {
        std::printf("FAIL: optimizer returned no result (%d frames)\n", frames);
        return 1;
    }

    const double best_a = sys.result().best_x.size() > 0 ? sys.result().best_x[0] : 0.0;
    const double best_b = sys.result().best_x.size() > 1 ? sys.result().best_x[1] : 0.0;
    const double best_f = sys.result().best_fitness.empty()
                              ? 1e9 : sys.result().best_fitness[0];
    std::printf("  frames=%d best=(%.6f, %.6f) f=%.3e\n",
                frames, best_a, best_b, best_f);

    if (best_f > 1e-3 || std::fabs(best_a - 1.0) > 0.05 ||
        std::fabs(best_b - 1.0) > 0.05) {
        std::printf("FAIL: did not converge to Rosenbrock optimum\n");
        return 1;
    }

    // 4. FitnessRecord must mirror the run's verdict + running=false.
    bool record_ok = false;
    reg.view<exd::sim::FitnessRecord>().each(
        [&](exd::ecs::Entity, const exd::sim::FitnessRecord& rec) {
            record_ok = !rec.running && rec.evaluations > 0 &&
                        std::fabs(rec.best_fitness - best_f) < 1e-9;
        });
    if (!record_ok) {
        std::printf("FAIL: FitnessRecord not published correctly\n");
        return 1;
    }

    std::printf("OK: generic optimization converged + published "
                "(f=%.3e, a=%.4f, b=%.4f)\n", best_f, best_a, best_b);
    return 0;
}
