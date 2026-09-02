// ShapeWorkshopSystem — parametric primitive gallery.
// Pure recipe dispatch lives in the anonymous namespace so the mapping
// spec → generate_*_mesh is headless-testable (see shape_workshop_test.cpp).
#include <exd/sim/shape_workshop_system.hpp>

#include "shape_recipes.hpp"

#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/renderable.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/components/unlit_material.hpp>
#include <exd/render/systems/imgui_system.hpp>

#include <imgui.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace exd::sim {

namespace {

/// Position slot i: two back/front rows, 2D shapes standing upright,
/// 3D shapes sitting on the ground plane.
math::Vec3f slot_position(int i, ShapeKind kind) {
    const float x = (static_cast<float>(i % 4) - 1.5f) * 2.9f;
    const float z = (i / 4 >= 1) ? 2.0f : -2.0f;
    const float y = is_2d_kind(kind) ? 2.0f : 1.2f;
    return {x, y, z};
}

} // namespace

// ── Public / frame ─────────────────────────────────────────────────

void ShapeWorkshopSystem::update(ecs::Registry& registry, double) {
    reg_ = &registry;
    ensure_entities(registry);

    for (size_t i = 0; i < shapes_.size(); ++i) {
        const ShapeWorkshopSpec& spec = registry.get<ShapeWorkshopSpec>(shapes_[i]);
        if (!same_spec(spec, last_specs_[i])) {
            rebuild_mesh(registry, shapes_[i], spec);
            last_specs_[i] = spec;
        }
    }
}

void ShapeWorkshopSystem::ensure_entities(ecs::Registry& registry) {
    if (entities_ready_) return;

    for (int i = 0; i < starter_count_; ++i)
        add_shape(registry, starter_shape_spec(i));

    if (!panel_added_) {
        auto panel = registry.create("ShapeWorkshopPanel");
        registry.emplace<render::ImGuiPanelComponent>(panel, "Shape Workshop",
            [this] { draw_panel(); });
        panel_added_ = true;
    }

    std::printf("[ShapeWorkshop] %d starter shapes ready\n", starter_count_);
    entities_ready_ = true;
}

void ShapeWorkshopSystem::add_shape(ecs::Registry& registry, const ShapeWorkshopSpec& spec) {
    const int i = static_cast<int>(shapes_.size());
    auto e = registry.create("Shape." + std::to_string(i));
    registry.emplace<render::Transform>(e, slot_position(i, spec.kind));
    registry.emplace<render::RenderableComponent>(e, 0u);
    registry.emplace<render::RenderTechnique_Unlit>(e);
    registry.emplace<render::UnlitMaterial>(e, math::Quat{1.0f, 1.0f, 1.0f, 1.0f});
    registry.emplace<ShapeWorkshopSpec>(e, spec);
    shapes_.push_back(e);
    last_specs_.push_back(spec);
    mesh_handles_.push_back(0u);
    rebuild_mesh(registry, e, spec);
}

void ShapeWorkshopSystem::rebuild_mesh(ecs::Registry& registry, ecs::Entity e,
                                       const ShapeWorkshopSpec& spec) {
    render::Mesh mesh = build_shape_mesh(spec);
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::printf("[ShapeWorkshop] build_shape_mesh returned empty mesh (kind %d)\n",
                    static_cast<int>(spec.kind));
        return;
    }

    // Mesh-handle index = position of this entity in shapes_.
    size_t slot = shapes_.size();
    for (size_t i = 0; i < shapes_.size(); ++i)
        if (shapes_[i].id == e.id && shapes_[i].gen == e.gen) slot = i;
    if (slot == shapes_.size()) return;

    const uint32_t new_handle = ctx_.mesh_manager.create(mesh);
    if (mesh_handles_[slot] != 0) ctx_.mesh_manager.destroy(mesh_handles_[slot]);
    mesh_handles_[slot] = new_handle;
    registry.get<render::RenderableComponent>(e).mesh = new_handle;

    std::printf("[ShapeWorkshop] rebuilt slot %zu: kind %d, %zu verts, %zu tris\n",
                slot, static_cast<int>(spec.kind),
                mesh.vertices.size(), mesh.indices.size() / 3);
}

// ── Panel ──────────────────────────────────────────────────────────

void ShapeWorkshopSystem::draw_panel() {
    if (!reg_) return;
    if (shapes_.empty()) return;
    if (selected_slot_ >= static_cast<int>(shapes_.size())) selected_slot_ = 0;

    auto& spec = reg_->get<ShapeWorkshopSpec>(shapes_[selected_slot_]);
    ImGui::Text("Primitive shapes — extropian-geometry recipes");
    ImGui::Separator();

    if (ImGui::BeginCombo("Shape", kShapeKindNames[static_cast<int>(spec.kind)])) {
        for (int k = 0; k < static_cast<int>(ShapeKind::Count); ++k) {
            if (ImGui::Selectable(kShapeKindNames[k], spec.kind == static_cast<ShapeKind>(k)))
                spec.kind = static_cast<ShapeKind>(k);
        }
        ImGui::EndCombo();
    }

    const bool is2d = is_2d_kind(spec.kind);
    ImGui::Text("Slot %d / %d", selected_slot_, static_cast<int>(shapes_.size()));
    if (ImGui::Button("Add shape")) {
        // Deferred: appended next update via a pending flag is overkill;
        // add inline — structural mutation is safe outside view iteration.
        auto& reg = *reg_;
        ShapeWorkshopSpec sp = starter_shape_spec(static_cast<int>(shapes_.size()) % 8);
        sp.kind = spec.kind; sp.color = spec.color;
        add_shape(reg, sp);
        last_specs_.push_back(sp);
        selected_slot_ = static_cast<int>(shapes_.size()) - 1;
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove last") && shapes_.size() > 1) {
        reg_->destroy(shapes_.back());
        shapes_.pop_back();
        last_specs_.pop_back();
        if (mesh_handles_.back() != 0) ctx_.mesh_manager.destroy(mesh_handles_.back());
        mesh_handles_.pop_back();
        if (selected_slot_ >= static_cast<int>(shapes_.size()))
            selected_slot_ = static_cast<int>(shapes_.size()) - 1;
    }
    ImGui::Separator();

    const int kMinSeg = 8, kMaxSeg = 128;
    const float kMinR = 0.05f;

    if (spec.kind == ShapeKind::RoundedRect2D) {
        ImGui::SliderFloat("Width", &spec.size_x, 0.5f, 4.0f, "%.2f");
        ImGui::SliderFloat("Height", &spec.size_y, 0.5f, 4.0f, "%.2f");
        ImGui::SliderFloat("Corner radius", &spec.corner_radius, 0.0f, 1.5f, "%.2f");
    } else if (spec.kind == ShapeKind::Box3D) {
        ImGui::SliderFloat("Size X", &spec.size_x, 0.2f, 4.0f, "%.2f");
        ImGui::SliderFloat("Size Y", &spec.size_y, 0.2f, 4.0f, "%.2f");
        ImGui::SliderFloat("Size Z", &spec.size_z, 0.2f, 4.0f, "%.2f");
    } else if (spec.kind == ShapeKind::Ellipsoid3D) {
        ImGui::SliderFloat("Radius X", &spec.size_x, 0.2f, 3.0f, "%.2f");
        ImGui::SliderFloat("Radius Y", &spec.size_y, 0.2f, 3.0f, "%.2f");
        ImGui::SliderFloat("Radius Z", &spec.size_z, 0.2f, 3.0f, "%.2f");
    } else if (spec.kind == ShapeKind::Cylinder3D || spec.kind == ShapeKind::Cone3D ||
               spec.kind == ShapeKind::Capsule3D) {
        ImGui::SliderFloat("Radius", &spec.radius, kMinR, 3.0f, "%.2f");
        ImGui::SliderFloat("Height", &spec.size_z, 0.2f, 4.0f, "%.2f");
    } else if (spec.kind == ShapeKind::Sphere3D) {
        ImGui::SliderFloat("Radius", &spec.radius, kMinR, 3.0f, "%.2f");
    } else if (spec.kind == ShapeKind::Torus3D) {
        ImGui::SliderFloat("Tube radius", &spec.inner_radius, 0.05f, 1.5f, "%.2f");
        ImGui::SliderFloat("Ring radius", &spec.radius, 0.2f, 3.0f, "%.2f");
    } else if (spec.kind == ShapeKind::Star2D) {
        ImGui::SliderFloat("Outer radius", &spec.radius, kMinR, 3.0f, "%.2f");
        ImGui::SliderFloat("Inner radius", &spec.inner_radius, kMinR, 2.5f, "%.2f");
        ImGui::SliderInt("Points", &spec.points, 3, 12);
    } else {
        // Circle / Ring
        ImGui::SliderFloat("Radius", &spec.radius, kMinR, 3.0f, "%.2f");
        if (spec.kind == ShapeKind::Ring2D)
            ImGui::SliderFloat("Inner radius", &spec.inner_radius, kMinR, 2.5f, "%.2f");
    }

    if (spec.kind != ShapeKind::RoundedRect2D && spec.kind != ShapeKind::Box3D &&
        spec.kind != ShapeKind::Ellipsoid3D)
        ImGui::SliderInt("Segments", &spec.segments, kMinSeg, kMaxSeg);

    ImGui::Separator();
    float rgba[4] = {spec.color.w, spec.color.x, spec.color.y, spec.color.z};
    if (ImGui::ColorEdit4("Color", rgba)) {
        // geometry convention: w=R, x=G, y=B, z=A
        spec.color = math::Quat{rgba[0], rgba[1], rgba[2], rgba[3]};
    }

    if (ImGui::Button("Reset shape")) spec = starter_shape_spec(static_cast<int>(shapes_.size()) % 8);
    ImGui::TextWrapped("Click a shape in the 3D view to select it; 1/2/3 = "
                       "translate / rotate / scale gizmo.");
}

} // namespace exd::sim
