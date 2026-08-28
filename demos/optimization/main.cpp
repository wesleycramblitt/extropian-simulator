// ─────────────────────────────────────────────────────────────────────
// Demo: analytical turbine + live CMA-ES optimization (system
// composition). The reference for multi-system coupling.
//
// The systems never talk to each other: OptimizationSystem locates the
// turbine by its TurbineSpec component and writes the design through the
// registry; TurbineSystem rebuilds/animates whatever the component says.
// Swap in a real solver system later by writing the same component.
//
// Run: build/extropian-sim-optimize   (Z toggles fly camera)
// ─────────────────────────────────────────────────────────────────────
#include <demo_app.hpp>

#include <exd/ecs/system_graph.hpp>
#include <exd/sim/optimization_system.hpp>
#include <exd/sim/turbine_system.hpp>

namespace {

class OptimizeDemo final : public DemoApp {
public:
    OptimizeDemo() : DemoApp("Extropian Sim — CMA-ES Turbine Optimization") {}

protected:
    void register_sim_systems(exd::ecs::SystemGraph& graph) override {
        // Order within the Simulation phase is load-bearing: the
        // optimizer writes TurbineSpec before TurbineSystem consumes it.
        graph.add<exd::sim::OptimizationSystem>(exd::ecs::SystemPhase::Simulation);
        graph.add<exd::sim::TurbineSystem>(exd::ecs::SystemPhase::Simulation,
                                           graphics());
    }
};

} // namespace

int main() {
    OptimizeDemo app;
    return app.run();
}
