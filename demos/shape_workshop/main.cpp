// ─────────────────────────────────────────────────────────────────────
// Demo: Shape Workshop — parametric primitive gallery + 3D gizmo editing.
//
// The default, copyable template for "custom geometry from recipes":
//   • ShapeWorkshopSystem (exd::sim) owns one Shape.N entity per slot with
//     a ShapeWorkshopSpec; the ImGui panel drives the recipe parameters
//     and meshes rebuild live from extropian-geometry generators.
//   • Render-side scene editing: PickerSystem + SelectionSystem pick
//     entities, Gizmo3DSystem (3D gizmo meshes from extropian-geometry)
//     translates/rotates/scales the selection.
//   • CameraModeSystem starts in Orbit (drag to spin, scroll to zoom).
//
// Controls:
//   Z             fly camera / UI mode toggle (FPS uses WASD + mouse)
//   UI mode:      click select, 1/2/3 = translate / rotate / scale gizmo
//   Shift+click   multi-select
//   G             grid on/off, X wireframe
// ─────────────────────────────────────────────────────────────────────
#include <demo_app.hpp>

#include <exd/ecs/system_graph.hpp>
#include <exd/render/camera_builder.hpp>
#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/camera_mode.hpp>
#include <exd/render/components/gizmo3d.hpp>
#include <exd/render/components/grid.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/interaction/picker.hpp>
#include <exd/render/interaction/selection.hpp>
#include <exd/render/systems/gizmo3d_system.hpp>
#include <exd/render/systems/grid_system.hpp>
#include <exd/sim/shape_workshop_system.hpp>

#include <SDL3/SDL.h>
#include <imgui.h>

#include <memory>

namespace {

class ShapeWorkshopDemo final : public DemoApp {
public:
    ShapeWorkshopDemo()
        : DemoApp("Extropian Sim — Shape Workshop"),
          picker_(gfx_.mesh_manager),
          gizmo_(gfx_) {}

protected:
    void register_sim_systems(exd::ecs::SystemGraph& graph) override {
        shapes_ = std::make_unique<exd::sim::ShapeWorkshopSystem>(graphics());
        graph.add_ref(exd::ecs::SystemPhase::Simulation, shapes_.get());

        // Render-side scene editing tooling (duck-typed systems via adapters).
        gizmo_adapter_ = std::make_unique<SystemAdapter<exd::render::Gizmo3DSystem>>(
            std::move(gizmo_));
        graph.add_ref(exd::ecs::SystemPhase::Simulation, gizmo_adapter_.get());

        grid_adapter_ = std::make_unique<SystemAdapter<exd::render::GridSystem>>(
            exd::render::GridSystem(graphics(), &window()));
        graph.add_ref(exd::ecs::SystemPhase::RenderPreparation, grid_adapter_.get());
    }

    void on_startup() override {
        DemoApp::on_startup();

        // Ground reference grid (uniform spacing, draws through the unlit pass).
        auto grid = registry().create("GroundGrid");
        registry().emplace<exd::render::GridComponent>(grid, 2.0f);
        registry().emplace<exd::render::Transform>(grid);

        // Editor tool settings — gizmo mode + anything the host wants.
        auto tools = registry().create("Tools");
        registry().emplace<exd::render::GizmoModeComponent>(tools);
        tools_ = tools;

        // Start in Orbit mode: UI cursor available for selection + panel.
        for (auto e : registry().view<exd::render::CameraModeController>()) {
            auto& ctl = registry().get<exd::render::CameraModeController>(e);
            ctl.mode = exd::render::CameraMode::Orbit;
            ctl.orbit_target = {0.0f, 1.0f, 0.0f};
            ctl.orbit_distance = 16.0f;
            break;
        }
    }

    void on_update(float dt) override {
        bool capture = false;
        if (ImGui::GetCurrentContext())
            capture = ImGui::GetIO().WantCaptureMouse;

        // While a gizmo drag is live or the cursor sits over ImGui, keep the
        // orbit camera from also reacting to the drag.
        for (auto e : registry().view<exd::render::CameraModeController>()) {
            registry().get<exd::render::CameraModeController>(e).lock_movement =
                capture || gizmo_.is_dragging();
        }

        // Gizmo mode keys (1/2/3).
        if (window().was_key_pressed(SDL_SCANCODE_1))
            registry().get<exd::render::GizmoModeComponent>(tools_).mode = exd::render::GizmoMode::Translate;
        if (window().was_key_pressed(SDL_SCANCODE_2))
            registry().get<exd::render::GizmoModeComponent>(tools_).mode = exd::render::GizmoMode::Rotate;
        if (window().was_key_pressed(SDL_SCANCODE_3))
            registry().get<exd::render::GizmoModeComponent>(tools_).mode = exd::render::GizmoMode::Scale;

        DemoApp::on_update(dt);   // runs the graph (camera + sim + render)

        // ── Scene-space pointer interaction (UI mode only, not over ImGui) ──
        int w = 1, h = 1; float aspect = 1.0f;
        window().get_dimensions(w, h, aspect);
        float mx = 0.f, my = 0.f;
        const uint32_t btn = SDL_GetMouseState(&mx, &my);
        const bool click = (btn & SDL_BUTTON_LMASK) && !(prev_mouse_ & SDL_BUTTON_LMASK);
        const bool held  = (btn & SDL_BUTTON_LMASK);
        prev_mouse_ = btn;

        exd::render::Camera camera;
        for (auto e : registry().view<exd::render::CameraComponent,
                                      exd::render::Transform>()) {
            camera = exd::render::make_camera(
                registry().get<exd::render::Transform>(e),
                registry().get<exd::render::CameraComponent>(e), aspect);
            break;
        }

        if (!capture) {
            if (gizmo_.is_dragging()) {
                if (!held) gizmo_.on_pointer_release();
                else       gizmo_.on_pointer_drag(registry(), camera, mx, my, (float)w, (float)h);
            } else if (click) {
                if (!gizmo_.on_pointer_press(registry(), camera, mx, my, (float)w, (float)h)) {
                    auto hit = picker_.pick(registry(), camera, mx, my, (float)w, (float)h);
                    const bool shift = window().keyboard_state &&
                                       window().keyboard_state[SDL_SCANCODE_LSHIFT];
                    selection_.handle_click(registry(), hit, shift);
                }
            }
            if (gizmo_.is_dragging() && !held) gizmo_.on_pointer_release();
            gizmo_.update_hover(registry(), camera, mx, my, (float)w, (float)h);
        }
    }

private:
    std::unique_ptr<exd::sim::ShapeWorkshopSystem> shapes_;
    exd::render::PickerSystem    picker_;
    exd::render::SelectionSystem selection_;
    exd::render::Gizmo3DSystem   gizmo_;
    std::unique_ptr<SystemAdapter<exd::render::Gizmo3DSystem>> gizmo_adapter_;
    std::unique_ptr<SystemAdapter<exd::render::GridSystem>>    grid_adapter_;
    exd::ecs::Entity tools_ = {};
    uint32_t prev_mouse_ = 0;
};

} // namespace

int main() {
    ShapeWorkshopDemo app;
    return app.run();
}
