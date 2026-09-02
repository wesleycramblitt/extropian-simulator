#pragma once
// ─────────────────────────────────────────────────────────────────────
// ShapeWorkshopSpec — ECS component carrying one parametric primitive
// shape generated from extropian-geometry's recipe descriptors. Plain
// data only (POD): satisfies the exd::ecs::Component concept.
//
// Writers:   ShapeWorkshopSystem panel (and the camera-modes demo, which
//            registers a small starter gallery)
// Readers:   ShapeWorkshopSystem (mesh rebuild on change)
// Entity:    each "Shape.N" entity created by ShapeWorkshopSystem.
// ─────────────────────────────────────────────────────────────────────
#include <exd/math/quat.hpp>

#include <cstdint>

namespace exd::sim {

/// Recipe kind dispatched by ShapeWorkshopSystem's mesh builder.
enum class ShapeKind : uint8_t {
    Circle2D      = 0,
    RoundedRect2D = 1,
    Star2D        = 2,
    Ring2D        = 3,
    Box3D         = 4,
    Sphere3D      = 5,
    Cylinder3D    = 6,
    Cone3D        = 7,
    Torus3D       = 8,
    Capsule3D     = 9,
    Ellipsoid3D   = 10,
    Count         = 11,
};

/// Human-readable kind names (panel + logs). Indexed by ShapeKind.
constexpr const char* kShapeKindNames[static_cast<int>(ShapeKind::Count)] = {
    "Circle (2D)",   "Rounded rect (2D)", "Star (2D)",    "Ring (2D)",
    "Box (3D)",      "Sphere (3D)",       "Cylinder (3D)","Cone (3D)",
    "Torus (3D)",    "Capsule (3D)",      "Ellipsoid (3D)",
};

struct ShapeWorkshopSpec {
    ShapeKind kind = ShapeKind::Sphere3D;

    // Shared recipe parameters. Not every field applies to every kind;
    // the panel only exposes the ones the active kind uses.
    float radius       = 1.0f;    // circle/star/ring/cylinder/cone/sphere/capsule/torus
    float inner_radius = 0.5f;    // star inner / ring inner / torus minor radius
    float size_x       = 1.0f;    // rect width / box x / ellipsoid rx
    float size_y       = 1.0f;    // rect height / box y / ellipsoid ry
    float size_z       = 1.0f;    // box z / cylinder/cone height / capsule height / ellipsoid rz
    float corner_radius = 0.2f;   // rounded-rect corner radius
    int   segments     = 48;      // circle/ring/star/cylinder/cone fidelity (slices)
    int   points       = 5;       // star points
    math::Quat color   = {0.8f, 0.3f, 0.2f, 1.0f}; // geometry convention: w=R, x=G, y=B, z=A
};

/// Free-function equality for dirty checks (component stays POD).
inline bool same_spec(const ShapeWorkshopSpec& a, const ShapeWorkshopSpec& b) {
    return a.kind          == b.kind &&
           a.radius        == b.radius &&
           a.inner_radius  == b.inner_radius &&
           a.size_x        == b.size_x &&
           a.size_y        == b.size_y &&
           a.size_z        == b.size_z &&
           a.corner_radius == b.corner_radius &&
           a.segments      == b.segments &&
           a.points        == b.points &&
           a.color.x       == b.color.x && a.color.y == b.color.y &&
           a.color.z       == b.color.z && a.color.w == b.color.w;
}

inline bool is_2d_kind(ShapeKind kind) {
    return kind >= ShapeKind::Circle2D && kind <= ShapeKind::Ring2D;
}

} // namespace exd::sim
