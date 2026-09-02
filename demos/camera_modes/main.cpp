// ─────────────────────────────────────────────────────────────────────
// Demo: Camera Modes — the unified mobile-camera controller.
//
// One Camera entity, one CameraModeController: FPS flight, turntable
// orbit, ground walk (z-clamped to an eye height), orthographic 2D editing
// and runtime FOV — all driven by camera_mode_system (extropian-render)
// and switchable at runtime.
//
// Controls / UI:
//   Panel:    mode buttons + FOV slider
//   1/2/3/4   FPS / Orbit / Walk / Ortho2D
//   Orbit:    left-drag orbit, scroll zoom, middle-drag pan
//   Ortho2D:  left-drag pan, scroll zoom
//   Fps/Walk: WASD + mouse look (QE adds vertical flight in FPS only)
//   Z         fly camera / UI mode toggle
// ─────────────────────────────────────────────────────────────────────
#include <demo_app.hpp>

#include <exd/ecs/system_graph.hpp>
#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/camera_mode.hpp>
#include <exd/render/components/grid.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/systems/grid_system.hpp>
#include <exd/render/systems/imgui_system.hpp>
#include <exd/sim/shape_workshop_system.hpp>

#include <SDL3/SDL.h>
#include <imgui.h>

#include <cstdio>
#include <memory>

namespace {

constexpr float kPiOver180 = 3.14159265358979323846f / 180.0f;

class CameraModesDemo final : public DemoApp {
public:
    CameraModesDemo() : DemoApp("Extropian Sim — Camera Modes") {}

protected:
    void register_sim_systems(exd::ecs::SystemGraph& graph) override {
        // A few parametric shapes so orbit/walk/ortho2d have interesting
        // targets — demonstrates cross-demo reuse of the shape library system.
        shapes_ = std::make_unique<exd::sim::ShapeWorkshopSystem>(graphics(), 5);
        graph.add_ref(exd::ecs::SystemPhase::Simulation, shapes_.get());

        grid_adapter_ = std::make_unique<SystemAdapter<exd::render::GridSystem>>(
            exd::render::GridSystem(graphics(), &window()));
        graph.add_ref(exd::ecs::SystemPhase::RenderPreparation, grid_adapter_.get());
    }

    void on_startup() override {
        DemoApp::on_startup();

        auto grid = registry().create("GroundGrid");
        registry().emplace<exd::render::GridComponent>(grid, 2.0f);
        registry().emplace<exd::render::Transform>(grid);

        // Start in Orbit around the shape cluster.
        for (auto e : registry().view<exd::render::CameraModeController>()) {
            auto& ctl = registry().get<exd::render::CameraModeController>(e);
            ctl.mode = exd::render::CameraMode::Orbit;
            ctl.orbit_target = {0.0f, 1.2f, 0.0f};
            set_mode_plumbing(exd::render::CameraMode::Orbit);
            break;
        }

        auto panel = registry().create("CameraModesPanel");
        registry().emplace<exd::render::ImGuiPanelComponent>(panel, "Camera Modes",
            [this] { draw_panel(); });
    }

    void on_update(float dt) override {
        // Hotkeys: 1=FPS 2=Orbit 3=Walk 4=Ortho2D
        if (window().was_key_pressed(SDL_SCANCODE_1)) set_mode(exd::render::CameraMode::Fps);
        if (window().was_key_pressed(SDL_SCANCODE_2)) set_mode(exd::render::CameraMode::Orbit);
        if (window().was_key_pressed(SDL_SCANCODE_3)) set_mode(exd::render::CameraMode::Walk);
        if (window().was_key_pressed(SDL_SCANCODE_4)) set_mode(exd::render::CameraMode::Ortho2D);

        DemoApp::on_update(dt);
    }

private:
    /// Find the live camera controller (first match, same convention as the
    /// camera system itself).
    exd::render::CameraModeController* camera_ctl() {
        for (auto e : registry().view<exd::render::CameraModeController>())
            return &registry().get<exd::render::CameraModeController>(e);
        return nullptr;
    }

    exd::render::CameraComponent* camera_comp() {
        for (auto e : registry().view<exd::render::CameraComponent>())
            return &registry().get<exd::render::CameraComponent>(e);
        return nullptr;
    }

    /// Switch the controller mode and keep the platform cursor mode in sync
    /// (captured for FPS/Walk, visible for Orbit/Ortho2D).
    void set_mode(exd::render::CameraMode mode) {
        set_mode_plumbing(mode);
    }

    void set_mode_plumbing(exd::render::CameraMode mode) {
        if (auto* ctl = camera_ctl()) ctl->mode = mode;
        window().set_input_mode((mode == exd::render::CameraMode::Fps ||
                                 mode == exd::render::CameraMode::Walk)
                                    ? exd::core::InputMode::FPS
                                    : exd::core::InputMode::UI);
    }

    void draw_panel() {
        using exd::render::CameraMode;
        auto* ctl = camera_ctl();
        if (!ctl) return;
        auto* cam = camera_comp();

        ImGui::Text("Camera controller — extropian-render");
        ImGui::Separator();
        int mode = static_cast<int>(ctl->mode);
        const char* modes[] = {"FPS (fly)", "Orbit", "Walk (ground)", "Ortho2D"};
        bool changed = false;
        for (int i = 0; i < 4; ++i) {
            if (ImGui::RadioButton(modes[i], mode == i)) { mode = i; changed = true; }
        }
        if (changed) set_mode(static_cast<CameraMode>(mode));

        ImGui::Separator();
        if (cam) {
            float fov_deg = cam->fov_y_radians / kPiOver180;
            if (ImGui::SliderFloat("FOV [deg]", &fov_deg, 20.0f, 120.0f, "%.0f"))
                cam->fov_y_radians = fov_deg * kPiOver180;
        }

        if (ctl->mode == CameraMode::Orbit) {
            ImGui::Separator();
            ImGui::Text("Orbit");
            ImGui::SliderFloat("Distance", &ctl->orbit_distance, ctl->min_distance, ctl->max_distance, "%.1f");
            ImGui::SliderFloat("Target Y", &ctl->orbit_target.y, -5.0f, 8.0f, "%.2f");
            ImGui::SliderFloat("Azimuth [deg]", &ctl->azimuth, -180.0f, 180.0f, "%.0f");
            ImGui::SliderFloat("Elevation [deg]", &ctl->elevation, -80.0f, 80.0f, "%.0f");
        } else if (ctl->mode == CameraMode::Ortho2D) {
            ImGui::Separator();
            ImGui::SliderFloat("Zoom", &ctl->ortho_zoom, 0.05f, 100.0f, "%.2f");
        } else {
            ImGui::Separator();
            ImGui::SliderFloat("Move speed", &ctl->move_speed, 1.0f, 80.0f, "%.1f");
            if (ctl->mode == CameraMode::Walk)
                ImGui::SliderFloat("Eye height", &ctl->walk_eye_height, 0.2f, 8.0f, "%.2f");
        }

        ImGui::Separator();
        ImGui::TextWrapped("Orbit: left-drag spin, scroll zoom, middle-drag pan.\n"
                           "Ortho2D: left-drag pan, scroll zoom.\n"
                           "Keys 1..4 switch modes; Z toggles fly/UI cursor.");
    }

    std::unique_ptr<exd::sim::ShapeWorkshopSystem> shapes_;
    std::unique_ptr<SystemAdapter<exd::render::GridSystem>> grid_adapter_;
};

} // namespace

int main() {
    CameraModesDemo app;
    return app.run();
}
