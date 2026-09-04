// ─────────────────────────────────────────────────────────────────────
// extropian-simulator-workspace — the interactive simulation workspace.
//
// One entry point that boots the workspace UI/UX skeleton: window,
// render pipeline (camera → cubemap sky → ImGui host), and an ECS
// seeded with placeholder domain state for the four prototyping axes:
//
//     CAD design → live simulation → visualization → optimization
//
// STUB: every domain is a placeholder component on its own entity. The
// real workspace layers (extropian-spatial-ui document/dashboard
// pipeline, interactive extropian-geometry editing, and the exd::sim
// systems for each domain) are not ready yet; they will replace these
// stubs as they land.
//
// Controls: Z toggles fly camera (WASD + mouse) / UI mode.
// ─────────────────────────────────────────────────────────────────────
#include <exd/app/application.hpp>
#include <exd/core/window_state.hpp>
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/ecs/system_graph.hpp>
#include <exd/ecs/view.hpp>
#include <exd/math/quat.hpp>
#include <exd/math/vec3.hpp>

#include <exd/render/graphics/graphics_context.hpp>
#include <exd/render/systems/camera_mode_system.hpp>
#include <exd/render/systems/cubemap_system.hpp>
#include <exd/render/systems/imgui_system.hpp>
#include <exd/render/systems/polygon_mode_system.hpp>
#include <exd/render/systems/primitive_mesh_system.hpp>
#include <exd/render/systems/render_system.hpp>

#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/camera_mode.hpp>
#include <exd/render/components/cubemap.hpp>
#include <exd/render/components/environment.hpp>
#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/transform.hpp>

#include <imgui.h>

#include <cstdio>
#include <memory>
#include <utility>

using namespace exd;

// ── Placeholder domain state (stub) ─────────────────────────────────
// POD components, one per prototyping axis. The real workspace systems
// (exd::sim + spatial-ui + geometry) replace these when they land.
struct CadDesignStub {     // CAD design under edit (placeholder)
    int    param_count = 0;    // parametric design params
    float  scale = 1.0f;       // design size factor
    bool   dirty = false;      // design changed since last solve
};

struct LiveSimulationStub {  // real-time simulation state (placeholder)
    int   status = 0;         // 0 idle, 1 running, 2 done
    float wind_speed = 8.0f;  // m/s, editable later
    float progress = 0.0f;    // 0..1 solve progress
};

struct VisualizationStub {   // visualization state (placeholder)
    int   mode = 0;           // 0 unset, 1 field, 2 streamlines, 3 iso
    float colormap_t = 0.5f;  // transfer-function anchor
};

struct OptimizationStub {    // optimization study state (placeholder)
    int    generations = 0;
    double best_fitness = 0.0;
    bool   running = false;
};

// ── Host shell: window + render pipeline + ImGui on a SystemGraph ──
// Duck-typed render systems (update(Registry&, double) without deriving
// from ecs::ISystem) run through SystemAdapter wrappers.
template <typename T>
class SystemAdapter final : public ecs::ISystem {
public:
    explicit SystemAdapter(T&& sys) : sys_(std::move(sys)) {}
    void update(ecs::Registry& registry, double dt) override {
        sys_.update(registry, dt);
    }
private:
    T sys_;
};

class WorkspaceApp final : public exd::app::Application {
public:
    WorkspaceApp()
        : Application(exd::app::WindowDesc{
              .title = "Extropian Simulator — Interactive Workspace",
              .width = 1600, .height = 900}) {}

protected:
    void on_startup() override {
        // ── Scene: camera, daylight cubemap sky, lighting ──
        auto cam = reg_.create("Camera");
        reg_.emplace<render::Transform>(cam, math::Vec3f{0.0f, 12.0f, 15.0f});
        reg_.emplace<render::CameraComponent>(cam);
        reg_.emplace<render::CameraModeController>(cam);
        window().set_input_mode(exd::core::InputMode::UI);

        auto sky = reg_.create("Sky");
        auto& cm = reg_.emplace<render::CubeMapComponent>(sky);
        cm.name = "Daylight";
        reg_.emplace<render::Transform>(sky);
        reg_.emplace<render::RenderTechnique_CubeMap>(sky);

        auto lighting = reg_.create("Lighting");
        reg_.emplace<render::SceneLighting>(lighting,
            math::Vec3f{0.2f, 0.2f, 0.2f},   // ambient
            math::Vec3f{0.5f, 1.0f, 0.3f},   // sun_direction
            math::Vec3f{1.0f, 1.0f, 1.0f});  // sun_color

        // ── Seed the ECS for the four prototyping axes (stubs) ──
        cad_ = reg_.create("CadDesign");
        reg_.emplace<CadDesignStub>(cad_, 12, 1.0f, true);

        sim_ = reg_.create("LiveSimulation");
        reg_.emplace<LiveSimulationStub>(sim_, 0, 8.0f, 0.0f);

        viz_ = reg_.create("Visualization");
        reg_.emplace<VisualizationStub>(viz_, 0, 0.5f);

        opt_ = reg_.create("OptimizationStudy");
        reg_.emplace<OptimizationStub>(opt_, 0, 0.0, false);

        std::printf("[Workspace] ECS seeded: CadDesign, LiveSimulation, "
                    "Visualization, OptimizationStudy (stubs)\n");

        // ── Panel: shows the seeded workspace state ──
        auto panel = reg_.create("WorkspacePanel");
        reg_.emplace<render::ImGuiPanelComponent>(
            panel, "Interactive Workspace", [this]() { draw_panel(); });

        // ── Pipeline: Input → RenderPreparation → Render ──
        imgui_ = std::make_unique<render::ImGuiSystem>(gfx_, &window());

        camera_adapter_ = std::make_unique<SystemAdapter<render::CameraModeSystem>>(
            render::CameraModeSystem(&window()));
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

        graph_.add<render::RenderSystem>(ecs::SystemPhase::Render, gfx_, &window());
        graph_.add_ref(ecs::SystemPhase::Render, imgui_.get());

        std::printf("[Workspace] Scene ready: %zu systems registered.\n",
                    graph_.count());
    }

    void on_update(float dt) override {
        if (window().input_mode != prev_input_mode_) {
            window().reset_mouse_delta();
            prev_input_mode_ = window().input_mode;
        }

        const exd::app::EventState& evs = window().events();
        for (int i = 0; i < evs.num_events; ++i)
            imgui_->process_event(evs.events[i]);

        graph_.update(reg_, dt);
        window().reset_mouse_delta();
    }

    void on_shutdown() override {
        imgui_->shutdown();
        imgui_.reset();
        graph_.clear();
        std::printf("[Workspace] Shutdown.\n");
    }

private:
    void draw_panel() {
        ImGui::TextUnformatted("Extropian Simulator — interactive workspace");
        ImGui::TextDisabled("STUB: spatial-ui document pipeline + interactive "
                            "geometry not ready yet");
        ImGui::Separator();

        if (reg_.has<CadDesignStub>(cad_)) {
            const auto& d = reg_.get<CadDesignStub>(cad_);
            ImGui::Text("[CAD ] CadDesign: %d params, scale %.2f%s",
                        d.param_count, d.scale, d.dirty ? " (dirty)" : "");
        }
        if (reg_.has<LiveSimulationStub>(sim_)) {
            const auto& s = reg_.get<LiveSimulationStub>(sim_);
            ImGui::Text("[SIM ] LiveSimulation: status %d, wind %.1f m/s, "
                        "progress %.0f%%", s.status, s.wind_speed, s.progress * 100.0f);
        }
        if (reg_.has<VisualizationStub>(viz_)) {
            const auto& v = reg_.get<VisualizationStub>(viz_);
            ImGui::Text("[VIZ ] Visualization: mode %d, colormap t=%.2f",
                        v.mode, v.colormap_t);
        }
        if (reg_.has<OptimizationStub>(opt_)) {
            const auto& o = reg_.get<OptimizationStub>(opt_);
            ImGui::Text("[OPT ] OptimizationStudy: %d generations, best f=%.4f%s",
                        o.generations, o.best_fitness, o.running ? " (running)" : "");
        }

        ImGui::Separator();
        ImGui::TextDisabled("Next: wire exd::sim systems + spatial-ui dashboards "
                            "into these entities.");
    }

    render::GraphicsContext gfx_;
    ecs::Registry reg_;
    ecs::SystemGraph graph_;
    ecs::Entity cad_ = {}, sim_ = {}, viz_ = {}, opt_ = {};
    std::unique_ptr<SystemAdapter<render::CameraModeSystem>> camera_adapter_;
    std::unique_ptr<SystemAdapter<render::CubeMapSystem>> cubemap_adapter_;
    std::unique_ptr<SystemAdapter<render::PolygonModeSystem>> poly_adapter_;
    std::unique_ptr<SystemAdapter<render::PrimitiveMeshSystem>> prim_adapter_;
    std::unique_ptr<render::ImGuiSystem> imgui_;
    exd::core::InputMode prev_input_mode_{exd::core::InputMode::FPS};
};

int main() {
    WorkspaceApp app;
    return app.run();
}
