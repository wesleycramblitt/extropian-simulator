#pragma once
// ─────────────────────────────────────────────────────────────────────
// TurbineSystem — parametric wind-turbine rotor built with
// extropian-geometry (exd::geometry :: TurbineDefinition / BladeRow).
//
// Owns one ECS entity carrying the turbine mesh (Transform +
// RenderableComponent, positioned with the rotor center just above the
// origin) plus an ImGuiPanelComponent panel with live parameter sliders.
// The mesh is rebuilt on the GPU only when a parameter actually changes.
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/render/graphics/graphics_context.hpp>

namespace exd::sim {

/// Live-editable turbine (rotor) parameters.
struct TurbineParams {
    int   blade_count = 3;       // blades per rotor
    float radius      = 4.0f;    // blade tip radius              [m]
    float hub_radius  = 0.35f;   // root (hub) radius             [m]
    float axial_chord = 1.0f;    // LE→TE axial extent at root    [m]
    float tip_taper   = 0.55f;   // TE axial extent at tip / root [-]
    float pitch_deg   = 2.0f;    // collective pitch              [deg]
    float twist_deg   = 22.0f;   // root-to-tip twist ramp        [deg]
    float thickness   = 0.12f;   // thickness-to-chord ratio      [t/c]
    float rpm         = 20.0f;   // visual rotor speed            [rpm]
    float yaw_deg     = 0.0f;    // rotor-plane yaw               [deg]

    // Hub / center body (extropian-geometry HubShape).
    int   hub_shape   = 0;       // 0 Spinner, 1 Bullet, 2 Cylinder,
                                 // 3 Tapered, 4 FlatDisk
    float hub_nose    = 0.60f;   // hub extent forward of rotor plane [m]
    float hub_aft     = 0.40f;   // hub extent behind rotor plane     [m]
};

class TurbineSystem final : public ecs::ISystem {
public:
    explicit TurbineSystem(render::GraphicsContext& ctx) : ctx_(ctx) {}

    void update(ecs::Registry& registry, double dt) override;

    /// Const access for diagnostics (the panel edits params_ directly).
    [[nodiscard]] const TurbineParams& params() const { return params_; }

private:
    // ECS create-on-first-update (entities + panel).
    void ensure_entities(ecs::Registry& registry);
    // Regenerate the CPU mesh, upload, and point the entity's
    // RenderableComponent at the new handle (destroys the old GPU mesh).
    void rebuild_mesh(ecs::Registry& registry);
    // ImGui panel body (registered via ImGuiPanelComponent).
    void draw_panel();

    // World-space height of the rotor center above the ground plane.
    float hub_height() const { return 0.6f + params_.radius; }

    render::GraphicsContext& ctx_;
    ecs::Entity entity_ = {};
    bool entities_ready_ = false;
    TurbineParams params_;
    TurbineParams last_params_;          // dirty check for rebuilds
    uint32_t mesh_handle_ = 0;
    float spin_deg_ = 0.0f;              // accumulated rotor spin
    bool panel_added_ = false;
};

} // namespace exd::sim
