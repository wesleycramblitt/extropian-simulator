#pragma once
// ─────────────────────────────────────────────────────────────────────
// SimulatorApp — the Extropian Simulator application.
// Declared here, implemented in simulator_app.cpp; main.cpp only boots it.
// ─────────────────────────────────────────────────────────────────────
#include <exd/app/application.hpp>
#include <exd/ecs/registry.hpp>

#include <memory>

namespace exd::render {
class CameraSystem;
class GridSystem;
class PolygonModeSystem;
class PrimitiveMeshSystem;
class RenderSystem;
class ImGuiSystem;
} // namespace exd::render

namespace exd::sim { class TurbineSystem; }

class SimulatorApp : public exd::app::Application {
public:
    SimulatorApp();
    ~SimulatorApp() override;

protected:
    void on_startup() override;
    void on_update(float dt) override;
    void on_shutdown() override;

private:
    exd::ecs::Registry reg_;
    std::unique_ptr<exd::render::CameraSystem>       camera_sys_;
    std::unique_ptr<exd::render::GridSystem>         grid_sys_;
    std::unique_ptr<exd::render::PolygonModeSystem>  poly_sys_;
    std::unique_ptr<exd::render::PrimitiveMeshSystem> prim_sys_;
    std::unique_ptr<exd::render::RenderSystem>       render_sys_;
    std::unique_ptr<exd::render::ImGuiSystem>        imgui_;
    std::unique_ptr<exd::sim::TurbineSystem>         turbine_sys_;

    /// Last input mode seen; used to flush mouse deltas on mode switches
    /// so entering fly mode never yanks the camera.
    exd::core::InputMode prev_input_mode_{exd::core::InputMode::FPS};
};
