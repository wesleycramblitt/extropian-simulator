#pragma once
// ─────────────────────────────────────────────────────────────────────
// DemoApp — the shared host shell for the demo executables. This is the
// *template* a researcher copies to build their own application on top
// of the exd::sim library: it owns the registry and the SystemGraph,
// wires the render pipeline (camera → cubemap → primitives → render →
// ImGui), and asks the subclass only for the sim systems to register.
//
// Frame pipeline (SystemGraph phases — insertion order within a phase):
//   Input             CameraModeSystem (FPS fly camera by default; Z toggles UI/fly mode)
//   Simulation        ← demo hook register_sim_systems() (sim physics)
//   RenderPreparation CubeMapSystem, PolygonModeSystem, PrimitiveMeshSystem
//   Render            RenderSystem (scene) → ImGuiSystem (panels)
//
// Some exd-render systems (CameraSystem, CubeMapSystem, ...) are
// duck-typed (they expose update(Registry&,double) but do not derive
// from ecs::ISystem); SystemAdapter wraps them so everything still runs
// through the SystemGraph.
//
// ImGui event feeding is app-level: SDL events are pushed into
// ImGuiSystem before the graph runs so panels register clicks in time.
// ─────────────────────────────────────────────────────────────────────
#include <exd/app/application.hpp>
#include <exd/core/window_state.hpp>
#include <exd/ecs/registry.hpp>
#include <exd/ecs/view.hpp>
#include <exd/ecs/system_graph.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/render/systems/camera_mode_system.hpp>
#include <exd/render/systems/cubemap_system.hpp>
#include <exd/render/systems/imgui_system.hpp>
#include <exd/render/systems/polygon_mode_system.hpp>
#include <exd/render/systems/primitive_mesh_system.hpp>

#include <memory>
#include <utility>

/// Adapts a duck-typed system (update(Registry&, double) but not an
/// exd::ecs::ISystem subclass) so it can be registered in a SystemGraph.
template <typename T>
class SystemAdapter final : public exd::ecs::ISystem {
public:
    explicit SystemAdapter(T&& sys) : sys_(std::move(sys)) {}
    void update(exd::ecs::Registry& registry, double dt) override {
        sys_.update(registry, dt);
    }
private:
    T sys_;
};

class DemoApp : public exd::app::Application {
public:
    explicit DemoApp(const char* title);
    ~DemoApp() override;

    exd::ecs::Registry& registry() { return reg_; }
    exd::render::GraphicsContext& graphics() { return gfx_; }

protected:
    void on_startup() override;
    void on_update(float dt) override;
    void on_shutdown() override;

    /// Demo hook: register the demo's sim systems into the graph.
    /// Convention: SystemPhase::Simulation, insertion order = execution
    /// order (e.g. optimization before turbine so specs land first).
    virtual void register_sim_systems(exd::ecs::SystemGraph& graph) = 0;

    exd::render::GraphicsContext gfx_;
    exd::ecs::Registry reg_;
    exd::ecs::SystemGraph graph_;

private:
    std::unique_ptr<SystemAdapter<exd::render::CameraModeSystem>>    camera_adapter_;
    std::unique_ptr<SystemAdapter<exd::render::CubeMapSystem>>      cubemap_adapter_;
    std::unique_ptr<SystemAdapter<exd::render::PolygonModeSystem>>  poly_adapter_;
    std::unique_ptr<SystemAdapter<exd::render::PrimitiveMeshSystem>> prim_adapter_;
    std::unique_ptr<exd::render::ImGuiSystem>                        imgui_;

    /// Last input mode seen; used to flush mouse deltas on mode switches
    /// so entering fly mode never yanks the camera.
    exd::core::InputMode prev_input_mode_{exd::core::InputMode::FPS};
};
