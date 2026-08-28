// DemoApp implementation — scene, render pipeline, and ImGui host.
//
// Base environment:
//   • fly camera: Z toggles free-fly (WASD + mouse) vs UI mode
//   • Daylight cubemap sky
//   • Scene-wide lighting
//   • ImGuiSystem (from extropian-render): event-driven ImGui host;
//     panels are ECS entities — any system emplaces an
//     ImGuiPanelComponent and it shows up automatically.
//
// All systems run through the ecs::SystemGraph; exd-app only provides
// the window + loop. Sim systems are added by the demo subclass through
// register_sim_systems().
#include <demo_app.hpp>

#include <exd/math/quat.hpp>
#include <exd/math/vec3.hpp>

#include <exd/render/systems/camera_system.hpp>
#include <exd/render/systems/cubemap_system.hpp>
#include <exd/render/systems/imgui_system.hpp>
#include <exd/render/systems/polygon_mode_system.hpp>
#include <exd/render/systems/primitive_mesh_system.hpp>
#include <exd/render/systems/render_system.hpp>

#include <exd/render/components/camera_controller.hpp>
#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/cubemap.hpp>
#include <exd/render/components/environment.hpp>
#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/transform.hpp>

#include <imgui.h>

#include <cstdio>

using namespace exd;

DemoApp::DemoApp(const char* title)
    : app::Application(app::WindowDesc{.title = title, .width = 1600, .height = 900}) {}

DemoApp::~DemoApp() = default;

void DemoApp::on_startup() {
    // ── Camera: starts at a vantage over the turbine; Z toggles
    // free-fly (FPS mode: WASD + mouse look) vs UI mode (panels).
    const math::Vec3f cam_pos{0.0f, 12.0f, 15.0f};

    auto cam = reg_.create("Camera");
    reg_.emplace<render::Transform>(cam, cam_pos);
    reg_.emplace<render::CameraComponent>(cam);
    reg_.emplace<render::CameraController>(cam);

    // Start in UI mode so panels are clickable; Z switches to fly mode.
    window().set_input_mode(exd::core::InputMode::UI);

    // ── Daylight cubemap sky: CubeMapSystem loads the texture on the
    // first frame so the cubemap pass has a full sky to draw. ──
    auto sky = reg_.create("Sky");
    auto& cm = reg_.emplace<render::CubeMapComponent>(sky);
    cm.name = "Daylight";
    reg_.emplace<render::Transform>(sky);
    reg_.emplace<render::RenderTechnique_CubeMap>(sky);

    // ── Scene-wide lighting for lit materials. ──
    auto lighting = reg_.create("Lighting");
    reg_.emplace<render::SceneLighting>(lighting,
        math::Vec3f{0.2f, 0.2f, 0.2f},   // ambient
        math::Vec3f{0.5f, 1.0f, 0.3f},   // sun_direction
        math::Vec3f{1.0f, 1.0f, 1.0f});  // sun_color

    // ── Render pipeline through the SystemGraph (phase order; insertion
    // order within a phase): input first, sim systems, materialization,
    // then the draw passes and finally the ImGui overlay. ──
    imgui_ = std::make_unique<render::ImGuiSystem>(gfx_, &window());

    // ── Render pipeline through the SystemGraph (phase order; insertion
    // order within a phase): input first, sim systems, materialization,
    // then the draw passes and finally the ImGui overlay. Duck-typed
    // render systems go in through SystemAdapter wrappers. ──
    camera_adapter_ = std::make_unique<SystemAdapter<render::CameraSystem>>(
        render::CameraSystem(&window()));
    graph_.add_ref(ecs::SystemPhase::Input, camera_adapter_.get());

    cubemap_adapter_ = std::make_unique<SystemAdapter<render::CubeMapSystem>>(
        render::CubeMapSystem(gfx_, &window()));
    graph_.add_ref(ecs::SystemPhase::RenderPreparation, cubemap_adapter_.get());

    poly_adapter_ = std::make_unique<SystemAdapter<render::PolygonModeSystem>>(
        render::PolygonModeSystem(&window()));
    graph_.add_ref(ecs::SystemPhase::RenderPreparation, poly_adapter_.get());

    prim_adapter_ = std::make_unique<SystemAdapter<render::PrimitiveMeshSystem>>(
        render::PrimitiveMeshSystem(gfx_, &window()));
    graph_.add_ref(ecs::SystemPhase::RenderPreparation, prim_adapter_.get());

    register_sim_systems(graph_);     // demo hook — Simulation phase

    graph_.add<render::RenderSystem>(ecs::SystemPhase::Render, gfx_, &window());
    graph_.add_ref(ecs::SystemPhase::Render, imgui_.get());

    std::printf("[DemoApp] Scene ready: camera -> sky cubemap, ImGui "
                "host, %zu systems registered.\n",
                graph_.count());
}

void DemoApp::on_update(float dt) {
    // Entering fly mode must not yank the camera: mouse deltas
    // accumulate during UI mode (CameraSystem only consumes them in
    // FPS mode) and the cursor-capture switch can emit a synthetic
    // motion event — flush everything on the transition frame so the
    // first FPS frame starts from a clean delta.
    if (window().input_mode != prev_input_mode_) {
        window().reset_mouse_delta();
        prev_input_mode_ = window().input_mode;
    }

    // Feed the raw SDL event stream to ImGui before the graph runs, so
    // panels see this frame's clicks. Camera input is consumed by
    // CameraSystem inside the graph (Input phase).
    const exd::app::EventState& evs = window().events();
    for (int i = 0; i < evs.num_events; ++i)
        imgui_->process_event(evs.events[i]);

    // Frame pipeline: Input → Simulation (demo systems) → RenderPreparation
    // → Render (scene + ImGui overlay).
    graph_.update(reg_, dt);

    // Keep UI-mode mouse-delta accumulation bounded for the fly camera.
    window().reset_mouse_delta();
}

void DemoApp::on_shutdown() {
    imgui_->shutdown();
    imgui_.reset();
    graph_.clear();   // drop system references before the adapters die
    std::printf("[DemoApp] Shutdown.\n");
}
