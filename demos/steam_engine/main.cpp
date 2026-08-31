// ─────────────────────────────────────────────────────────────────────
// Demo: steam engine — meshing, 0D solver run, optimization.
//
// Composition (all library recipes, no solver glue in the demo):
//   • SteamEngineSystem      — parametric steam-engine Assembly mesh
//     (regenerated per crank angle), background simulate_engine() run
//     (Rankine-lite), indicator-diagram samples, animation
//   • OptimizationSystem (engine-sim mode) — CMA-ES over boiler
//     pressure / cutoff / crank radius / bore; each candidate is a fast
//     0D engine simulation; the best design is written to EngineSpec
//
// ECS order is load-bearing: the optimizer writes EngineSpec before
// SteamEngineSystem consumes it.
//
// Run: build/extropian-sim-steam-engine [--auto-run]
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
#include <exd/sim/steam_engine_system.hpp>

#include <memory>

namespace {

class SteamEngineDemo final : public DemoApp {
public:
    explicit SteamEngineDemo(bool auto_run)
        : DemoApp("Extropian Sim — Steam Engine"),
          auto_run_(auto_run) {}

protected:
    void register_sim_systems(exd::ecs::SystemGraph& graph) override {
        // Simulation phase (order is load-bearing): the optimizer writes
        // EngineSpec before the engine consumes it; the dashboard feed
        // publishes domain state into the spatial-ui doc.
        opt_ = std::make_unique<exd::sim::OptimizationSystem>(
            exd::sim::ObjectiveModel::EngineSim);
        graph.add_ref(exd::ecs::SystemPhase::Simulation, opt_.get());
        engine_ = std::make_unique<exd::sim::SteamEngineSystem>(graphics());
        graph.add_ref(exd::ecs::SystemPhase::Simulation, engine_.get());

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
        if (!opt_started_) {
            opt_->start_optimization();
            opt_started_ = true;
        }
        if (opt_started_ && !opt_->is_running() && !run_started_) {
            engine_->start_run();
            run_started_ = true;
        }
    }

private:
    std::unique_ptr<exd::sim::OptimizationSystem> opt_;
    std::unique_ptr<exd::sim::SteamEngineSystem> engine_;
    std::unique_ptr<exd::scene_renderer::DocumentLoader> loader_;
    std::unique_ptr<exd::sim::DashboardFeedSystem> dash_;
    bool auto_run_ = false;
    bool opt_started_ = false;
    bool run_started_ = false;
    int frames_ = 0;
};

} // namespace

int main(int argc, char** argv) {
    const bool auto_run = argc > 1 && std::string_view(argv[1]) == "--auto-run";
    SteamEngineDemo app(auto_run);
    return app.run();
}
