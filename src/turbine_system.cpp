// TurbineSystem implementation — parametric rotor mesh + live panel.
// All design state lives in the TurbineSpec component on the entity;
// the system only keeps dirty-snapshot and animation state as members.
#include <exd/sim/turbine_system.hpp>

#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_ops.hpp>
#include <exd/geometry/turbine.hpp>

#include <exd/math/quat.hpp>
#include <exd/math/vec3.hpp>

#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/renderable.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/systems/imgui_system.hpp>

#include <imgui.h>

#include <array>
#include <cmath>
#include <cstdio>

namespace exd::sim {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float deg2rad(float d) { return d * kPi / 180.0f; }

bool same_spec(const TurbineSpec& a, const TurbineSpec& b) {
    return a.blade_count == b.blade_count &&
           a.radius      == b.radius      &&
           a.hub_radius  == b.hub_radius  &&
           a.axial_chord == b.axial_chord &&
           a.tip_taper   == b.tip_taper   &&
           a.pitch_deg   == b.pitch_deg   &&
           a.twist_deg   == b.twist_deg   &&
           a.thickness   == b.thickness   &&
           a.rpm         == b.rpm         &&
           a.yaw_deg     == b.yaw_deg     &&
           a.hub_shape   == b.hub_shape   &&
           a.hub_nose    == b.hub_nose    &&
           a.hub_aft     == b.hub_aft &&
           a.rotor_sense == b.rotor_sense;
}

/// Build the whole machine mesh in geometry space:
///   • rotor disc around the Z axis (blades in the XY plane)
///   • parametric hub / center body at the rotor center (shape selected in
///     the panel; radius follows the blade-root radius)
/// The mesh's rotor plane sits at z = 0; the ECS Transform lifts it to hub
/// height. Returns an empty mesh on failure.
render::Mesh build_turbine_mesh(const TurbineSpec& p) {
    using namespace exd::geometry;

    // ── Rotor: one BladeRow, straight LE at z=0, tapered TE ──
    BladeRow row;
    row.type = BladeRowType::Rotor;
    row.blade_count = {static_cast<float>(p.blade_count), 1.0f, 20.0f, "", false};
    row.leading_edge_hub     = {0.0f, p.hub_radius};
    row.leading_edge_shroud  = {0.0f, p.radius};
    row.trailing_edge_hub    = {p.axial_chord, p.hub_radius};
    row.trailing_edge_shroud = {p.axial_chord * p.tip_taper, p.radius};
    row.chordwise_points = 20;

    constexpr int kSections = 5;
    row.sections.reserve(kSections);
    for (int i = 0; i < kSections; ++i) {
        const float f = static_cast<float>(i) / static_cast<float>(kSections - 1);
        BladeSection sec;
        sec.span = f;
        const float stagger = p.pitch_deg + p.twist_deg * (1.0f - f);
        sec.stagger = {stagger, -90.0f, 90.0f, "deg", false};
        sec.inlet_metal_angle = {stagger + 25.0f, -90.0f, 90.0f, "deg", false};
        sec.exit_metal_angle  = {stagger - 10.0f, -90.0f, 90.0f, "deg", false};
        sec.max_thickness = {p.thickness, 0.001f, 0.9f, "t/c", false};
        row.sections.push_back(sec);
    }

    FlowPath flow;   // the blade builder only consults tip_clearance
    flow.tip_clearance = {0.01f, 0.0f, 0.02f, "m", false};

    MeshData rotor = generate_blade_row_mesh(row, flow, 48);
    if (rotor.vertices.empty()) return {};

    // ── Hub / center body: revolved, capped profile at the rotor center.
    // The hub radius tracks the blade-root radius so the hub grows with the
    // "Hub radius" slider and the blades stay attached to it. Shape selects
    // the machine kind (wind-turbine spinner, bullet nose, plain cylinder,
    // double-tapered spindle, or a thin flat disc).
    HubDefinition hub;
    // The panel index 0..4 maps onto the library's shape list starting at
    // HubShape::Spinner; HubShape::None is the library's "no hub" default.
    static_assert(static_cast<int>(HubShape::Spinner) == 1,
                  "turbine_system panel indices assume Spinner == 1");
    hub.shape        = static_cast<HubShape>(p.hub_shape + 1);
    hub.root_radius  = p.hub_radius;
    hub.front_length = p.hub_nose;
    hub.aft_length   = p.hub_aft;
    MeshData hub_mesh = generate_hub_mesh(hub, 48);

    if (hub_mesh.vertices.empty()) return rotor;
    const std::array<MeshData, 2> parts{rotor, hub_mesh};
    return merge_meshes(parts);
}

} // namespace

// ── Frame ───────────────────────────────────────────────────────────

void TurbineSystem::update(ecs::Registry& registry, double dt) {
    reg_ = &registry;
    ensure_entities(registry);

    const TurbineSpec& spec = registry.get<TurbineSpec>(entity_);

    // Live rebuild when a parameter changed (dirty check each frame).
    if (!same_spec(spec, last_spec_)) {
        rebuild_mesh(registry, spec);
        last_spec_ = spec;
    }

    // Rotor center just above the origin; spin around Z, yaw around Y.
    // Sense 0 = turbine (CCW viewed from upwind +Z), 1 = fan (CW).
    const float sense_dir = (spec.rotor_sense == 0) ? 1.0f : -1.0f;
    spin_deg_ += sense_dir * spec.rpm * 6.0f * static_cast<float>(dt);   // rpm -> deg/s
    if (spin_deg_ > 360.0f) spin_deg_ -= 360.0f;
    if (spin_deg_ <   0.0f) spin_deg_ += 360.0f;

    const math::Quat yaw  = math::Quat::from_axis_angle(
        math::Vec3f{0.0f, 1.0f, 0.0f}, deg2rad(spec.yaw_deg));
    const math::Quat spin = math::Quat::from_axis_angle(
        math::Vec3f{0.0f, 0.0f, 1.0f}, deg2rad(spin_deg_));

    auto& xform = registry.get<render::Transform>(entity_);
    xform.rotation = (yaw * spin).norm();
    xform.position = math::Vec3f{0.0f, hub_height(spec), 0.0f};
}

// ── Entities / panel ────────────────────────────────────────────────

void TurbineSystem::ensure_entities(ecs::Registry& registry) {
    if (entities_ready_) return;

    entity_ = registry.create("Turbine");
    registry.emplace<render::Transform>(entity_);
    registry.emplace<render::RenderableComponent>(entity_, 0u);
    registry.emplace<render::RenderTechnique_Mirror>(entity_);
    auto& spec = registry.emplace<TurbineSpec>(entity_);
    rebuild_mesh(registry, spec);   // uploads mesh + sets RenderableComponent
    last_spec_ = spec;

    if (!panel_added_) {
        auto panel = registry.create("TurbinePanel");
        registry.emplace<render::ImGuiPanelComponent>(panel, "Turbine",
            [this] { draw_panel(); });
        panel_added_ = true;
    }

    entities_ready_ = true;
}

void TurbineSystem::rebuild_mesh(ecs::Registry& registry, const TurbineSpec& spec) {
    render::Mesh mesh = build_turbine_mesh(spec);
    if (mesh.vertices.empty()) {
        std::printf("[Turbine] build_turbine_mesh returned empty mesh\n");
        return;
    }

    const uint32_t new_handle = ctx_.mesh_manager.create(mesh);
    if (mesh_handle_ != 0) ctx_.mesh_manager.destroy(mesh_handle_);
    std::printf("[Turbine] mesh built: %zu verts, %zu indices\n",
                mesh.vertices.size(), mesh.indices.size());
    mesh_handle_ = new_handle;

    // Point the entity at the fresh GPU mesh.
    registry.get<render::RenderableComponent>(entity_).mesh = new_handle;
}

// ── Panel ───────────────────────────────────────────────────────────

void TurbineSystem::draw_panel() {
    if (!reg_) return;
    auto& p = reg_->get<TurbineSpec>(entity_);
    ImGui::Text("Wind-turbine rotor — extropian-geometry");
    ImGui::Separator();

    ImGui::SliderInt("Blades", &p.blade_count, 2, 8);
    ImGui::SliderFloat("Radius [m]", &p.radius, 1.0f, 8.0f, "%.2f");
    ImGui::SliderFloat("Hub radius [m]", &p.hub_radius, 0.2f, 1.0f, "%.2f");
    ImGui::SliderFloat("Axial chord [m]", &p.axial_chord, 0.3f, 2.5f, "%.2f");
    ImGui::SliderFloat("Tip taper", &p.tip_taper, 0.35f, 1.0f, "%.2f");
    ImGui::Separator();
    ImGui::Text("Hub / center body");
    static const char* kHubShapes[] = {
        "Spinner (wind turbine)", "Bullet (high-speed)",
        "Cylinder (axial core)",  "Tapered (spindle)",
        "Flat disk (disc rotor)",
    };
    ImGui::Combo("Hub shape", &p.hub_shape, kHubShapes, IM_ARRAYSIZE(kHubShapes));
    const bool flat_disk = (p.hub_shape == 4);
    if (flat_disk) ImGui::BeginDisabled();
    ImGui::SliderFloat("Hub nose [m]", &p.hub_nose, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Hub aft [m]",   &p.hub_aft,  0.0f, 3.0f, "%.2f");
    if (flat_disk) ImGui::EndDisabled();
    ImGui::Separator();
    ImGui::SliderFloat("Pitch [deg]", &p.pitch_deg, -180.0f, 180.0f, "%.1f");
    ImGui::SliderFloat("Twist ramp [deg]", &p.twist_deg, -90.0f, 90.0f, "%.1f");
    ImGui::SliderFloat("Thickness t/c", &p.thickness, 0.06f, 0.35f, "%.3f");
    ImGui::Separator();
    ImGui::SliderFloat("RPM", &p.rpm, 0.0f, 120.0f, "%.0f");
    ImGui::SliderFloat("Yaw [deg]", &p.yaw_deg, -180.0f, 180.0f, "%.1f");
    static const char* kSense[] = {
        "Turbine (CCW from upwind)",
        "Fan (CW from upwind)",
    };
    ImGui::Combo("Rotor sense", &p.rotor_sense, kSense, IM_ARRAYSIZE(kSense));
    ImGui::TextWrapped("With the default blade camber (convex side at +theta), "
                       "CCW spin extracts energy (turbine); CW spin pushes air "
                       "(fan). Pitch can be any angle; |pitch|>90 flips the "
                       "sections.");

    ImGui::Separator();
    if (ImGui::Button("Reset")) p = TurbineSpec{};
    ImGui::TextWrapped("The mesh rebuilds in real time as you drag.");
}

} // namespace exd::sim
