#pragma once
// ─────────────────────────────────────────────────────────────────────
// ShapeWorkshopSystem — parametric primitive-shape gallery driven by
// extropian-geometry's 2D/3D recipe generators.
//
// Owning the ECS side of the "shape workshop": one "Shape.N" entity per
// slot carrying Transform + RenderableComponent + ShapeWorkshopSpec, plus
// an ImGui "ShapeWorkshopPanel" giving live parameter control. Meshes are
// rebuilt on the GPU only when a slot's spec changes between frames
// (canonical TurbineSystem pattern). Shapes render with the Unlit
// technique so the recipes' baked vertex colors show.
//
// Render-side editing (move/rotate/scale) is driven by the demo through
// exd::render::PickerSystem/SelectionSystem — this system
// owns shape *parameters*, not scene editing.
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/sim/components/shape_workshop.hpp>

#include <vector>

namespace exd::sim {

class ShapeWorkshopSystem final : public ecs::ISystem {
public:
    explicit ShapeWorkshopSystem(render::GraphicsContext& ctx, int starter_count = 8)
        : ctx_(ctx), starter_count_(starter_count) {}

    void update(ecs::Registry& registry, double dt) override;

    /// Number of shape slots currently owned (for demo glue / panels).
    int  slot_count() const { return static_cast<int>(shapes_.size()); }
    /// Entity of slot i (for demo selection landings).
    ecs::Entity slot_entity(int i) const { return shapes_[static_cast<size_t>(i)]; }

private:
    void ensure_entities(ecs::Registry& registry);
    void add_shape(ecs::Registry& registry, const ShapeWorkshopSpec& spec);
    void rebuild_mesh(ecs::Registry& registry, ecs::Entity e, const ShapeWorkshopSpec& spec);
    void draw_panel();

    render::GraphicsContext& ctx_;
    ecs::Registry* reg_ = nullptr;
    int  starter_count_ = 8;
    std::vector<ecs::Entity>        shapes_;
    std::vector<ShapeWorkshopSpec>  last_specs_;
    std::vector<uint32_t>           mesh_handles_;
    int  selected_slot_ = 0;
    bool entities_ready_ = false;
    bool panel_added_ = false;
};

} // namespace exd::sim
