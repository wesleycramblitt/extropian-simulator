// ─────────────────────────────────────────────────────────────────────
// Demo: real coupled FDM3 CFD — solve AND optimize the turbine design.
//
// Composition (all library recipes, no solver glue in the demo):
//   • OptimizationSystem (coupled-CFD mode) — CMA-ES over the design;
//     every candidate is a SHORT FDM3 coupled run on a worker thread
//     (12³ grid, ~0.7 s/eval, one at a time)
//   • SolverRunSystem   — "Solve" launches a FULL coupled run; the final
//     flow field is published as a speed slice + streamlines (viz cores)
//   • TurbineSystem     — parametric rotor mesh + live design panel
//
// ECS order is load-bearing: the optimizer writes TurbineSpec before
// TurbineSystem consumes it; SolverRunSystem reads the design at Solve time.
//
// Run: build/extropian-sim-turbine-solver [--auto-run]
//      (--auto-run starts a coupled optimization AND a full solver run
//       ~0.5 s after boot: smoke test of the whole pipeline)
//      Z toggles fly camera / UI mode
// ─────────────────────────────────────────────────────────────────────
#include <demo_app.hpp>

#include <exd/ecs/system_graph.hpp>
#include <exd/render/systems/ui_render_system.hpp>
#include <exd/scene_renderer/document_loader.hpp>
#include <exd/scene_renderer/systems/screen_widget_system.hpp>
#include <exd/scene_renderer/systems/systems.hpp>
#include <exd/sim/dashboard_feed_system.hpp>
#include <exd/sim/optimization_system.hpp>
#include <exd/sim/solver_run_system.hpp>
#include <exd/sim/turbine_system.hpp>

#include <memory>

namespace {

class TurbineSolverDemo final : public DemoApp {
public:
    explicit TurbineSolverDemo(bool auto_run)
        : DemoApp("Extropian Sim — Coupled FDM3 Turbine Solver"),
          auto_run_(auto_run) {}

protected:
    void register_sim_systems(exd::ecs::SystemGraph& graph) override {
        // ── Simulation phase (order is load-bearing) ──
        // Optimizer writes TurbineSpec → solver/turbine consume it → the
        // dashboard feed publishes domain state into the spatial-ui doc.
        opt_ = std::make_unique<exd::sim::OptimizationSystem>(
            exd::sim::ObjectiveModel::CoupledCfd);
        graph.add_ref(exd::ecs::SystemPhase::Simulation, opt_.get());
        solver_ = std::make_unique<exd::sim::SolverRunSystem>(graphics());
        graph.add_ref(exd::ecs::SystemPhase::Simulation, solver_.get());
        graph.add<exd::sim::TurbineSystem>(exd::ecs::SystemPhase::Simulation,
                                           graphics());

        // ── spatial-ui dashboard (composer-style resolution pipeline) ──
        loader_ = std::make_unique<exd::scene_renderer::DocumentLoader>(registry());
        dash_ = std::make_unique<exd::sim::DashboardFeedSystem>(*loader_);
        graph.add_ref(exd::ecs::SystemPhase::Simulation, dash_.get());

        int w = 1280, h = 720;
        float aspect = 1.0f;
        window().get_dimensions(w, h, aspect);
        graph.add<exd::scene_renderer::FontSystem>(exd::ecs::SystemPhase::Structural);
        graph.add<exd::scene_renderer::SizeSystem>(exd::ecs::SystemPhase::Structural);
        graph.add<exd::scene_renderer::LayoutSystem>(exd::ecs::SystemPhase::Layout,
                                                     loader_->hierarchy());
        auto& vf = graph.add<exd::scene_renderer::ViewportFitSystem>(
            exd::ecs::SystemPhase::Layout);
        vf.set_viewport(static_cast<float>(w), static_cast<float>(h));
        graph.add<exd::scene_renderer::MeshSystem>(
            exd::ecs::SystemPhase::RenderPreparation, graphics());
        graph.add<exd::scene_renderer::RelationSystem>(
            exd::ecs::SystemPhase::RenderPreparation, graphics());
        graph.add<exd::scene_renderer::RenderOrderSystem>(
            exd::ecs::SystemPhase::RenderPreparation);
        auto& sw = graph.add<exd::scene_renderer::ScreenWidgetSystem>(
            exd::ecs::SystemPhase::RenderPreparation);
        sw.set_viewport(static_cast<float>(w), static_cast<float>(h));
        graph.add<exd::scene_renderer::CameraSystem>(
            exd::ecs::SystemPhase::RenderPreparation);
        graph.add<exd::render::UIRenderSystem>(exd::ecs::SystemPhase::Render,
                                               graphics(), &window());
    }

    void on_update(float dt) override {
        DemoApp::on_update(dt);
        if (!auto_run_ || ++frames_ < 5) return;
        if (!auto_started_) {
            opt_->start_optimization();
            auto_started_ = true;
        }
        // Once the optimization run is done, solve the optimized best design.
        if (auto_started_ && !opt_->is_running() && !solve_started_) {
            solver_->solve();
            solve_started_ = true;
        }
    }

private:
    std::unique_ptr<exd::sim::OptimizationSystem> opt_;
    std::unique_ptr<exd::sim::SolverRunSystem> solver_;
    std::unique_ptr<exd::scene_renderer::DocumentLoader> loader_;
    std::unique_ptr<exd::sim::DashboardFeedSystem> dash_;
    bool auto_run_ = false;
    bool auto_started_ = false;
    bool solve_started_ = false;
    int frames_ = 0;
};

} // namespace

int main(int argc, char** argv) {
    const bool auto_run = argc > 1 && std::string_view(argv[1]) == "--auto-run";
    TurbineSolverDemo app(auto_run);
    return app.run();
}
