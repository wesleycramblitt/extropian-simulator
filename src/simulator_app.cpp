// SimulatorApp implementation — scene, systems, and UI host.
//
// Base environment:
//   • fly camera: Z toggles free-fly (WASD + mouse) vs UI mode
//   • reference ground grid + unit cube at the origin
//   • ImGuiSystem (from extropian-render): event-driven ImGui host;
//     panels are ECS entities — any system emplaces an
//     ImGuiPanelComponent and it shows up automatically.
//
// The ECS registry is owned here and systems are driven explicitly in
// on_update(); exd-app only provides the window + loop.
#include "simulator_app.hpp"

#include <exd/core/window_state.hpp>          // InputMode
#include <exd/math/mat4.hpp>
#include <exd/math/quat.hpp>
#include <exd/math/vec3.hpp>

#include <exd/render/graphics/graphics_context.hpp>
#include <exd/render/systems/camera_system.hpp>
#include <exd/render/systems/grid_system.hpp>
#include <exd/render/systems/imgui_system.hpp>
#include <exd/render/systems/polygon_mode_system.hpp>
#include <exd/render/systems/primitive_mesh_system.hpp>
#include <exd/render/systems/render_system.hpp>

#include "turbine_system.hpp"

#include <exd/render/components/camera_controller.hpp>
#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/cube.hpp>
#include <exd/render/components/grid.hpp>
#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/transform.hpp>

#include <imgui.h>

#include <cstdio>
#include <memory>

using namespace exd;

namespace {

// ── Graphics context shared by all render systems ────────
render::GraphicsContext gfx;

} // namespace

SimulatorApp::SimulatorApp()
    : app::Application(app::WindowDesc{
          .title = "Extropian Simulator",
          .width = 1600, .height = 900}) {}

SimulatorApp::~SimulatorApp() = default;

void SimulatorApp::on_startup() {
    // ── Camera: starts at a vantage over the origin; Z toggles
    // free-fly (FPS mode: WASD + mouse look) vs UI mode (panels).
    const math::Vec3f cam_pos{0.0f, 5.0f, 5.0f};

    auto cam = reg_.create("Camera");
    reg_.emplace<render::Transform>(cam, cam_pos);
    reg_.emplace<render::CameraComponent>(cam);
    reg_.emplace<render::CameraController>(cam);

    // Start in UI mode so panels are clickable; Z switches to fly mode.
    window().set_input_mode(exd::core::InputMode::UI);

    // ── Reference scene: ground grid + unit cube at the origin ──
    auto grid = reg_.create("Grid");
    reg_.emplace<render::GridComponent>(grid, 5.0f,
        math::Quat{0.4f, 0.4f, 0.4f, 0.4f});
    reg_.emplace<render::Transform>(grid);
    reg_.emplace<render::RenderTechnique_Lambertian>(grid);

    // ── Systems (fixed order = fixed frame pipeline) ──
    camera_sys_ = std::make_unique<render::CameraSystem>(&window());
    grid_sys_   = std::make_unique<render::GridSystem>(gfx, &window());
    poly_sys_   = std::make_unique<render::PolygonModeSystem>(&window());
    prim_sys_   = std::make_unique<render::PrimitiveMeshSystem>(gfx, &window());
    render_sys_ = std::make_unique<render::RenderSystem>(gfx, &window());
    imgui_      = std::make_unique<render::ImGuiSystem>(gfx, &window());
    turbine_sys_ = std::make_unique<sim::TurbineSystem>(gfx);

    // ── Panels: ECS entities with an ImGuiPanelComponent ──
    // Other systems register their own panels the same way — the
    // ImGui host draws one window per component and it just shows up.
    auto panel = reg_.create("SimulatorPanel");
    reg_.emplace<render::ImGuiPanelComponent>(panel, "Simulator", [this] {
        ImGui::Text("Extropian Simulator");
        ImGui::Separator();
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::Separator();
        ImGui::TextWrapped("Panels are ECS entities: emplace "
                           "render::ImGuiPanelComponent and the host "
                           "shows them automatically.");
    });

    std::printf("[Simulator] Scene ready: camera -> origin, grid, cube, "
                "ImGui host.\n");
}

void SimulatorApp::on_update(float dt) {
    // Entering fly mode must not yank the camera: mouse deltas
    // accumulate during UI mode (CameraSystem only consumes them in
    // FPS mode) and the cursor-capture switch can emit a synthetic
    // motion event — flush everything on the transition frame so the
    // first FPS frame starts from a clean delta.
    if (window().input_mode != prev_input_mode_) {
        window().reset_mouse_delta();
        prev_input_mode_ = window().input_mode;
    }

    // Frame pipeline: input → geometry → render → UI overlay
    camera_sys_->update(reg_, dt);     // free-fly: WASD + mouse look (Z toggles)
    window().reset_mouse_delta();      // keep UI-mode accumulation bounded
    grid_sys_->update(reg_, dt);
    poly_sys_->update(reg_, dt);
    prim_sys_->update(reg_, dt);    // GPU meshes for primitives
    turbine_sys_->update(reg_, dt); // turbine mesh + rotor spin (panel too)
    render_sys_->update(reg_, dt);  // clears + draws the scene

    // ImGui host: feed the raw SDL event stream buffered by the
    // window, then draw the registered panels on top of the scene.
    const exd::app::EventState& evs = window().events();
    for (int i = 0; i < evs.num_events; ++i)
        imgui_->process_event(evs.events[i]);
    imgui_->update(reg_, dt);
}

void SimulatorApp::on_shutdown() {
    imgui_->shutdown();
    imgui_.reset();
    camera_sys_.reset();
    render_sys_.reset();
    turbine_sys_.reset();
    prim_sys_.reset();
    poly_sys_.reset();
    grid_sys_.reset();
    std::printf("[Simulator] Shutdown.\n");
}
