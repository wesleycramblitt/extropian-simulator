// Extropian Simulator — main entry point
#include <exd/app/application.hpp>
#include <exd/app/mode.hpp>
#include <exd/app/system_graph.hpp>
#include <exd/app/window.hpp>

#include <exd/render/systems.hpp>
#include <exd/render/graphics_context.hpp>
#include <exd/render/components.hpp>

#include <exd/ecs/registry.hpp>
#include <exd/core/config.hpp>

#include <cstdio>

using namespace exd;

// ── Simulator modes ──────────────────────────────────────
enum SimMode { Setup = 0, Simulate = 1, Postprocess = 2 };

// ── Graphics context shared by all render systems ────────
static render::GraphicsContext gfx;

class SimulatorApp : public app::Application {
public:
    SimulatorApp() {
        modes().define({SimMode::Setup, "Setup", "Geometry and boundary conditions"});
        modes().define({SimMode::Simulate, "Simulate", "Run solver with live visualization"});
        modes().define({SimMode::Postprocess, "Postprocess", "Explore results and export"});
    }

protected:
    void on_configure(exd::core::Config& cfg) override {
        cfg.set_default("render.backend", "opengl");
    }

    void on_setup(exd::ecs::Registry& reg) override {
        // ── Camera (positioned outside the domain looking in) ──
        auto cam = reg.create("Camera");
        reg.emplace<render::Transform>(cam,
            math::Vec3f{0.0f, 45.0f, 200.0f},
            math::Quat{1.0f, 0.0f, 0.0f, 0.0f},
            math::Vec3f{1.0f, 1.0f, 1.0f});
        reg.emplace<render::Camera>(cam);
        reg.emplace<render::CameraController>(cam);

        // ── Grid ──
        auto grid = reg.create("Grid");
        reg.emplace<render::GridComponent>(grid, 50.0f,
            math::Quat{0.4f, 0.4f, 0.4f, 0.4f});
        reg.emplace<render::Transform>(grid);
        reg.emplace<render::RenderTechnique_Lambertian>(grid);

        std::printf("[Simulator] Scene initialized: camera + grid.\n");
    }

    void on_register_systems(app::SystemGraph& graph) override {
        auto* win = &this->window();

        // Always-on systems
        graph.add<render::RenderSystem>(gfx, win).always();
        graph.add<render::CameraSystem>(win).always();
        graph.add<render::PolygonModeSystem>(win).always();
        graph.add<render::GridSystem>(gfx, win).always();

        graph.build();
    }

    void on_update(double) override {
        // FPS display every 60 frames
        static int frame = 0;
        static auto last = std::chrono::steady_clock::now();
        if (++frame % 60 == 0) {
            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - last).count();
            std::printf("[Simulator] FPS: %.1f\n", 60.0f / elapsed);
            last = now;
        }
    }

    void on_mode_changed(int from, int to) override {
        std::printf("[Simulator] Mode: %d -> %d\n", from, to);
    }
};

int main(int argc, char** argv) {
    SimulatorApp app;
    return app.run();
}
