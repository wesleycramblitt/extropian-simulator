#pragma once
// ─────────────────────────────────────────────────────────────────────
// shape_recipes.hpp — pure spec→mesh mapping for the ShapeWorkshopSystem.
// Single source of truth shared by src/shape_workshop_system.cpp and the
// headless shape_workshop_test (mirrors coupled_run.hpp / engine_run.hpp).
//
// All functions are pure (no ECS/GL); they only call extropian-geometry's
// recipe generators.
// ─────────────────────────────────────────────────────────────────────
#include <exd/core/mesh_types.hpp>
#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/primitives3d.hpp>
#include <exd/sim/components/shape_workshop.hpp>

#include <array>

namespace exd::sim {

/// Build a mesh for a spec by dispatching onto the matching recipe.
inline exd::core::MeshData build_shape_mesh(const ShapeWorkshopSpec& spec) {
    using namespace exd::geometry;

    switch (spec.kind) {
    case ShapeKind::Circle2D:
        return generate_circle_mesh(CircleGeometry{
            .radius = spec.radius, .segments = static_cast<uint32_t>(spec.segments),
            .color = spec.color});
    case ShapeKind::RoundedRect2D:
        return generate_rounded_rect_mesh(RoundedRectangleGeometry{
            .size = {spec.size_x, spec.size_y, 0.0f},
            .radii = CornerRadii{.topLeft = spec.corner_radius,
                                 .topRight = spec.corner_radius,
                                 .bottomRight = spec.corner_radius,
                                 .bottomLeft = spec.corner_radius},
            .cornerSegments = 16,
            .color = spec.color});
    case ShapeKind::Star2D:
        return generate_star_mesh(StarGeometry{
            .outerRadius = spec.radius, .innerRadius = spec.inner_radius,
            .points = static_cast<uint32_t>(spec.points),
            .color = spec.color});
    case ShapeKind::Ring2D:
        return generate_ring_mesh(RingGeometry{
            .outerRadius = spec.radius, .innerRadius = spec.inner_radius,
            .segments = static_cast<uint32_t>(spec.segments),
            .color = spec.color});
    case ShapeKind::Box3D:
        return generate_box_mesh(BoxGeometry{
            .size = {spec.size_x, spec.size_y, spec.size_z}, .color = spec.color});
    case ShapeKind::Sphere3D:
        return generate_sphere_mesh(SphereGeometry{
            .radius = spec.radius,
            .latitudeSegments = static_cast<uint32_t>(spec.segments),
            .longitudeSegments = static_cast<uint32_t>(spec.segments) * 2,
            .construction = SphereConstruction::Uv,
            .generateNormals = true, .generateTexcoords = false,
            .color = spec.color});
    case ShapeKind::Cylinder3D:
        return generate_cylinder_mesh(CylinderGeometry{
            .radius = spec.radius, .height = spec.size_z,
            .slices = static_cast<uint32_t>(spec.segments), .capped = true,
            .color = spec.color});
    case ShapeKind::Cone3D:
        return generate_cone_mesh(ConeGeometry{
            .radius = spec.radius, .height = spec.size_z,
            .slices = static_cast<uint32_t>(spec.segments), .capped = true,
            .color = spec.color});
    case ShapeKind::Torus3D:
        return generate_torus_mesh(TorusGeometry{
            .majorRadius = spec.radius, .minorRadius = spec.inner_radius,
            .majorSegments = static_cast<uint32_t>(spec.segments),
            .minorSegments = 24,
            .color = spec.color});
    case ShapeKind::Capsule3D:
        return generate_capsule_mesh(CapsuleGeometry{
            .radius = spec.radius, .height = spec.size_z,
            .slices = static_cast<uint32_t>(spec.segments), .stacks = 16,
            .color = spec.color});
    case ShapeKind::Ellipsoid3D:
        return generate_ellipsoid_mesh(EllipsoidGeometry{
            .radii = {spec.size_x, spec.size_y, spec.size_z},
            .latitudeSegments = static_cast<uint32_t>(spec.segments),
            .longitudeSegments = static_cast<uint32_t>(spec.segments) * 2,
            .color = spec.color});
    default:
        return {};
    }
}

/// Starter spec for slot i (two rows of shapes).
inline ShapeWorkshopSpec starter_shape_spec(int i) {
    ShapeWorkshopSpec spec;
    switch (i) {
    case 0: spec.kind = ShapeKind::Circle2D; spec.radius = 1.4f; break;
    case 1: spec.kind = ShapeKind::Star2D;   spec.radius = 1.6f; spec.inner_radius = 0.7f; spec.points = 5; break;
    case 2: spec.kind = ShapeKind::RoundedRect2D; spec.size_x = 2.4f; spec.size_y = 1.6f; spec.corner_radius = 0.5f; break;
    case 3: spec.kind = ShapeKind::Ring2D;   spec.radius = 1.4f; spec.inner_radius = 0.85f; break;
    case 4: spec.kind = ShapeKind::Sphere3D; spec.radius = 1.1f; break;
    case 5: spec.kind = ShapeKind::Cylinder3D; spec.radius = 0.9f; spec.size_z = 2.2f; break;
    case 6: spec.kind = ShapeKind::Torus3D;  spec.radius = 1.2f; spec.inner_radius = 0.35f; break;
    case 7: spec.kind = ShapeKind::Cone3D;   spec.radius = 1.0f; spec.size_z = 2.4f; break;
    default: spec.kind = ShapeKind::Box3D;   spec.size_x = spec.size_y = spec.size_z = 1.0f; break;
    }
    static const std::array<math::Quat, 8> kPalette = {{
        {1.0f, 0.35f, 0.30f, 1.0f}, {0.30f, 0.75f, 1.00f, 1.0f},
        {0.25f, 0.85f, 0.45f, 1.0f}, {1.00f, 0.78f, 0.25f, 1.0f},
        {0.70f, 0.45f, 1.00f, 1.0f}, {1.00f, 0.55f, 0.85f, 1.0f},
        {0.55f, 0.90f, 0.90f, 1.0f}, {0.90f, 0.60f, 0.40f, 1.0f},
    }};
    spec.color = kPalette[static_cast<size_t>(i) % kPalette.size()];
    return spec;
}

} // namespace exd::sim
