#pragma once
// ─────────────────────────────────────────────────────────────────────
// TurbineSystem — parametric wind-turbine rotor built with
// extropian-geometry (exd::geometry :: TurbineDefinition / BladeRow).
//
// The system owns one ECS entity carrying the design (TurbineSpec) plus
// the render representation (Transform + RenderableComponent, positioned
// with the rotor center just above the origin) and an ImGuiPanelComponent
// panel with live parameter sliders. The mesh is rebuilt on the GPU only
// when the TurbineSpec component actually changes between frames.
//
// ECS contract (see AGENTS.md):
//   • reads/owns TurbineSpec on its own entity
//   • other systems (e.g. OptimizationSystem) drive the design by writing
//     TurbineSpec through the registry — never via calls into this system
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/sim/components/turbine.hpp>

namespace exd::sim {

class TurbineSystem final : public ecs::ISystem {
public:
    explicit TurbineSystem(render::GraphicsContext& ctx) : ctx_(ctx) {}

    void update(ecs::Registry& registry, double dt) override;

private:
    // ECS create-on-first-update (entities + panel).
    void ensure_entities(ecs::Registry& registry);
    // Regenerate the CPU mesh, upload, and point the entity's
    // RenderableComponent at the new handle (destroys the old GPU mesh).
    void rebuild_mesh(ecs::Registry& registry, const TurbineSpec& spec);
    // ImGui panel body (registered via ImGuiPanelComponent).
    void draw_panel();

    // World-space height of the rotor center above the ground plane.
    float hub_height(const TurbineSpec& spec) const { return 3.0f + spec.radius; }

    render::GraphicsContext& ctx_;
    ecs::Registry* reg_ = nullptr;       // bound on first update
    ecs::Entity entity_ = {};
    bool entities_ready_ = false;
    TurbineSpec last_spec_;              // dirty check for rebuilds
    uint32_t mesh_handle_ = 0;
    float spin_deg_ = 0.0f;              // accumulated rotor spin
    bool panel_added_ = false;
};

} // namespace exd::sim
