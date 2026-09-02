// ─────────────────────────────────────────────────────────────────────
// shape_workshop_test — headless verification of the ShapeWorkshopSystem
// recipe mapping (shape_recipes.hpp): every ShapeKind dispatch must
// produce a sane, colored, deterministic mesh.
//
// Compiles against src/shape_recipes.hpp only (no ECS, no GL).
// ─────────────────────────────────────────────────────────────────────
#include <exd/sim/components/shape_workshop.hpp>
#include "shape_recipes.hpp"

#include <cstdint>
#include <cstdio>

using exd::sim::ShapeKind;
using exd::sim::ShapeWorkshopSpec;
using exd::sim::build_shape_mesh;
using exd::sim::starter_shape_spec;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

} // namespace

int main() {
    // 1. Every reachable kind produces a non-empty triangle mesh.
    for (int k = 0; k < static_cast<int>(ShapeKind::Count); ++k) {
        ShapeWorkshopSpec spec;
        spec.kind = static_cast<ShapeKind>(k);
        auto mesh = build_shape_mesh(spec);
        char what[96];
        std::snprintf(what, sizeof(what), "kind %d mesh non-empty", k);
        check(!mesh.vertices.empty() && !mesh.indices.empty(), what);
        check((mesh.indices.size() % 3) == 0, "triangle indices");
    }

    // 2. Color is propagated into vertex color (geometry w=R convention).
    {
        ShapeWorkshopSpec spec;
        spec.kind = ShapeKind::Sphere3D;
        spec.color = {1.0f, 0.25f, 0.5f, 1.0f};  // R=1, G=0.25, B=0.5, A=1
        auto mesh = build_shape_mesh(spec);
        check(!mesh.vertices.empty(), "sphere has vertices");
        if (!mesh.vertices.empty()) {
            const auto& c = mesh.vertices[0].color;
            check(c.w == 1.0f && c.x == 0.25f && c.y == 0.5f && c.z == 1.0f,
                  "vertex color == spec.color (Quat convention)");
        }
    }

    // 3. Determinism: same spec twice → identical mesh.
    {
        ShapeWorkshopSpec spec = starter_shape_spec(5);
        auto a = build_shape_mesh(spec);
        auto b = build_shape_mesh(spec);
        check(a.vertices.size() == b.vertices.size(), "deterministic vertex count");
        check(a.indices.size()  == b.indices.size(),  "deterministic index count");
    }

    // 4. Default starter set: 8 variety-packed, valid slots.
    {
        bool all_valid = true;
        for (int i = 0; i < 8; ++i) {
            ShapeWorkshopSpec spec = starter_shape_spec(i);
            if (build_shape_mesh(spec).vertices.empty()) all_valid = false;
        }
        check(all_valid, "all 8 starter specs build");
        check(starter_shape_spec(0).kind == ShapeKind::Circle2D, "slot 0 is Circle2D");
        check(starter_shape_spec(7).kind == ShapeKind::Cone3D,   "slot 7 is Cone3D");
    }

    if (failures == 0) {
        std::printf("shape_workshop_test: all checks passed\n");
        return 0;
    }
    std::printf("shape_workshop_test: %d failure(s)\n", failures);
    return 1;
}
