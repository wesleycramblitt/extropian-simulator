// ─────────────────────────────────────────────────────────────────────
// Demo: analytical turbine (single system).
//
// The minimal "plug and play" example: the heap of this file IS the
// whole app. A researcher's own application follows the same shape —
// subclass DemoApp, register systems into the SystemGraph, done.
//
// Run: build/extropian-sim-turbine   (Z toggles fly camera)
// ─────────────────────────────────────────────────────────────────────
#include <demo_app.hpp>

#include <exd/ecs/system_graph.hpp>
#include <exd/sim/turbine_system.hpp>

namespace {

class TurbineDemo final : public DemoApp {
public:
    TurbineDemo() : DemoApp("Extropian Sim — Analytical Turbine") {}

protected:
    void register_sim_systems(exd::ecs::SystemGraph& graph) override {
        // One system: parametric rotor driven by its TurbineSpec
        // component (editable in the "Turbine" panel).
        graph.add<exd::sim::TurbineSystem>(exd::ecs::SystemPhase::Simulation,
                                           graphics());
    }
};

} // namespace

int main() {
    TurbineDemo app;
    return app.run();
}
