#include <exd/app/application.hpp>
#include <exd/app/ui_host.hpp>
#include <exd/app/mode.hpp>
#include <exd/app/system_graph.hpp>
#include <exd/render/systems.hpp>
#include <exd/render/graphics_context.hpp>
#include <exd/render/components.hpp>
#include <exd/ecs/registry.hpp>
// Solver disabled: #include <exd/solver/fluidx3d/fluidx3d_system.hpp>
// Solver disabled: #include <exd/solver/fluidx3d/components.hpp>

using namespace exd;

// ── Simulator modes ──────────────────────────────────────
namespace {
    enum SimMode { Setup = 0, Simulate = 1, Postprocess = 2 };
}

class SimulatorApp : public app::Application {
public:
    SimulatorApp() {
        modes().define({SimMode::Setup, "Setup", "Geometry, materials, boundary conditions"});
        modes().define({SimMode::Simulate, "Simulate", "Run solver with live visualization"});
        modes().define({SimMode::Postprocess, "Postprocess", "Explore results, export data"});
    }

protected:
    void on_configure(exd::core::Config& cfg) override {
        cfg.set_default("render.backend", "opengl");
        cfg.set_default("ui.backend", "rmlui");
        cfg.set_default("solver.default", "fluidx3d");
    }

    void on_setup(exd::ecs::Registry& registry) override {
        // ── Camera ──
        auto cam = registry.create("Camera");
        registry.emplace<render::Transform>(cam, math::Vec3{0, 45, 200});
        registry.emplace<render::Camera>(cam);
        registry.emplace<render::CameraController>(cam);
        registry.emplace<render::ReadOnly>(cam);

        // ── Grid ──
        auto grid = registry.create("Grid");
        registry.emplace<render::GridComponent>(grid, 50.0f, math::Quat{0.4f, 0.4f, 0.4f, 0.4f});
        registry.emplace<render::Transform>(grid);

        // ── Simulation entity ──
        auto sim = registry.create("WindTunnel");
//        registry.emplace<solver::fluidx3d::FluidX3DSolverConfig>(sim);
//        auto& phys = registry.emplace<solver::fluidx3d::FluidPhysics>(sim,
            0.02f,   // viscosity
            0.15f,   // streamwise velocity
            0,       // X-axis flow
            0, 0, 0  // volume forces
        );
//        auto& info = registry.emplace<solver::fluidx3d::SimulationInfo>(sim);

        // ── Domain box ──
        auto domain = registry.create("DomainBox");
//        registry.emplace<solver::fluidx3d::SimulationDomain>(domain, 250, 80, 128);
        registry.emplace<render::Transform>(domain, math::Vec3{-20, 80, 0});
        registry.emplace<render::RenderTechnique_Lambertian>(domain);
//        registry.emplace<solver::fluidx3d::SimulationReference>(domain, sim.id);

        // ── Volume field ──
        auto vol = registry.create("Volume");
//        registry.emplace<solver::fluidx3d::VolumeField>(vol);
//        registry.emplace<solver::fluidx3d::SimulationReference>(vol, sim.id);

        // ── Particle cloud ──
        auto particles = registry.create("Particles");
//        registry.emplace<solver::fluidx3d::ParticleCloud>(particles);
//        registry.emplace<solver::fluidx3d::SimulationReference>(particles, sim.id);
    }

    void on_register_systems(app::SystemGraph& graph) override {
        // Always-on systems
        graph.add<render::PolygonModeSystem>().always();

        // Setup mode systems
        graph.add<render::PrimitiveMeshSystem>(ctx_).in_mode(SimMode::Setup);
        graph.add<render::CubeMapSystem>(ctx_).in_mode(SimMode::Setup);
        graph.add<render::MeshAssetSystem>(ctx_).in_mode(SimMode::Setup);
        graph.add<render::GridSystem>(ctx_).in_mode(SimMode::Setup);

        // All-mode systems
        graph.add<render::RenderSystem>(ctx_).always();
        graph.add<render::CameraSystem>().always();

        // Simulate mode systems
//        graph.add<solver::fluidx3d::FluidX3DSystem>(ctx_).in_mode(SimMode::Simulate);

        graph.build();
    }

    void on_load_ui(app::IUIHost& ui) override {
        ui.load_stylesheet("assets/theme/simulator.rcss");
        auto doc = ui.load_document("assets/panels/solver_config.rml");
        if (doc) {
            doc->on("solve.run", [this](const auto&) { modes().request_transition(SimMode::Simulate); });
            doc->on("solve.stop", [this](const auto&) { modes().request_transition(SimMode::Setup); });
        }
    }

private:
    render::GraphicsContext ctx_;
};

int main(int argc, char** argv) {
    SimulatorApp app;
    return app.run();
}
