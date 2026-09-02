# Ecosystem Status and Required Additions

> What sibling Extropian repos provide today, what this repo relies on, and
> the concrete additions required of extropian-render and
> extropian-spatial-ui. This is the **living contract** for cross-repo work
> (`plan.md` Phase 0–1). Status verified against working trees, Aug 2026.

## 1. Quick Status Matrix

| Repo | Provides today | Simulator relies on | Gaps for this product |
|---|---|---|---|
| **extropian-core** | ECS (sparse-set registry, views, command buffer, hierarchy index), SystemGraph (working tree adds a phase enum; the app-level graph used by this repo exposes insertion order + `order_before`), math (vec/mat/quat/dualquat, bounds, raycast, color), FixedTimestep, RNG (PCG32), Config, JSON glue, VisualDocument/SemanticDocument types | Everything in `architecture.md` | No parallelism in SystemGraph (by design — cores own it); empty `exd::geom` (we add meshing in-repo); no ECS serialization (studies are JSON docs) |
| **extropian-render** | OpenGL renderer, multi-pass RenderSystem, techniques (lambertian/reflective/particle/volume/highlight; gizmo parts render as unlit overlay entities via `Gizmo3DSystem`), mesh/asset managers (static upload), CPU PickerSystem, `Gizmo3DSystem` (T/R/S on extropian-geometry gizmo meshes — unlit overlay parts), `CameraModeSystem`/`CameraModeController` (FPS/orbit/walk/ortho-2D), GLAD 4.6 (compute entry points loaded), headless *nominal* | Render backend + compute substrate (with additions) | See §3 — compute is entirely missing in practice |
| **extropian-spatial-ui** | geometry (2-D/3-D meshes, text w/ FreeType+HarfBuzz), layout engine (13 strategies), ui descriptor→mesh generators (35+), interaction (events, gestures, state machine, commands/undo, data bindings, 2-D hit), scene_renderer (VisualDocument→ECS runtime), all build + tests green | Entire workbench surface (widgets, dashboards) | No retained interactive widgets, no 3-D hit testing, no input routing — see §4; 2-D gizmo stack removed 2026-09 (gizmo editing lives in render) |
| **extropian-physics** | `ISolverPlugin`/`SolverManager` interface, `MaterialDatabase` (air/water/Al/steel/copper), mesh/field/bc structs (src-internal), I/O all stubs | Material database; mesh-quality idea; solver *interface pattern* | Public headers for mesh/field/BC types missing; io stubs (we implement our own exporters in-repo) |
| **extropian-app** | Window, modes, application base, input state | App shell, mode machinery | None — works today (the prototype uses it) |
| **extropian-solver-fluidx3d** | wrapper around FluidX3D-CLI (was partially implemented) | — | **Removed.** Simulator builds custom solvers (`ecosystem.md` §5); references already deleted from this repo |
| **extropian-composer** | VisualDocument→ECS demo app, TOML config (incl. `ai.*` sections), no AI code | Config pattern for LLM adapter settings | None required |
| **extropian-semantic-to-visual** | Retired/empty on disk (was AI→document pipeline) | — | Not used; LLM adapter lives in this repo (`ai/`), provider agnostic |

## 2. What This Repo Owns (no dependencies on siblings)

- All solver cores (`src/physics/`): BEMT, LBM, beam, turbulence.
- Optimization engine (`ecs/systems/optimization/` + cores).
- Meshing/voxelization.
- Post-processing logic (streamlines, slices, iso, probes, derived fields).
- Exporters (STL/STEP, VTK/CSV, report) — `src/io/`.
- Spec parsing (`ai/`) and auto-configuration rules.
- View models that project domain state into spatial-ui documents.

## 3. Required Additions — extropian-render (compute substrate)

The vendored GLAD 4.6 loader already exposes every entry point
(`glDispatchCompute`, SSBO, FBO, `glMapBufferRange`, `glTexImage3D`); no build
changes. Concrete work items (size-ordered):

1. **Compute-shader pipeline support.** `ShaderManager` currently compiles
   VS+FS pairs only. Add `get_or_load_compute(name, path)` with compute stage
   compile/link; `#version 430 core` for desktop GL (WebGL/Emscripten path is
   compute-incapable — guard with `__EMSCRIPTEN__`, desktop-only feature).
2. **Generic GPU buffer (`GpuBuffer`)** — `glBufferStorage` + SSBO binding +
   `glMapBufferRange(GL_MAP_PERSISTENT/COHERENT|READ|WRITE)` + orphan/resize;
   used by LBM fields, particle pools, force accumulators. (Mirrors the
   planned design in `docs/instancing.md`.)
3. **3-D texture source.** Concrete `ITextureSource` for `GL_TEXTURE_3D`
   (upload + `update_level` region updates). `TextureManager::update(handle,
   source)` already exists and calls `update_level(0,0)` — complete the path.
4. **FBO `RenderTarget`** per `docs/render-architecture.md` §3.3
   (RGBA16F HDR target, ping-pong, blit) — needed for post-processing and
   (optionally) slice composition.
5. **Volume pass completion.** `render_volume_pass` passes `proxy_mesh = 0`
   and `ray_march.frag` is a constant-color stub. Fix: proxy box mesh + real
   raymarching with 3-D texture sampling + transfer function uniform.
6. **Particle pass GPU path.** Variant of `ParticleRenderTechnique` that draws
   from an SSBO (solver-written positions) instead of per-frame CPU
   reinterleave + `glBufferSubData`.
7. **Headless wiring.** `IRenderer::create(Backend::Null)` should return the
   existing `NullRenderer` (currently returns `nullptr`) — enables full
   system-graph CI without a window.
8. **Multiviewport (v2).** Loop over cameras with per-viewport `glViewport` +
   clears — needed for multi-panel engineering views (optimize-inspect-live).

None of these change the render API contract for existing content; they extend
`GraphicsContext`-owned managers. The simulator's `compute/` layer is the
consumer and holds no raw GL.

## 4. Usage — extropian-spatial-ui (workbench)

The workbench is built on spatial-ui as-is (no retained-widget toolkit
exists or is planned there — widgets are descriptor→mesh generators). This
repo will:

- **Author panels as `VisualDocument`s** (prompt bar, study tree, variable
  inspector, fitness/constraint panels, plots) and run them through
  `scene_renderer::SpatialUiRuntime` into the shared registry (pattern proven
  by extropian-composer).
- **Wire interaction ourselves**: SDL events → `PointerEvent/KeyboardEvent`
  → `GestureRecognizer` → render `PickerSystem` → `resolve_hit` →
  `InteractionState` → our commands. Sliders etc. are visual meshes + our
  drag→value mapping through `CommandStack` + `IDocument` (the interaction
  module was built for exactly this wiring; nothing consumes it yet).
- **Use `exd::render::Gizmo3DSystem`** (T/R/S on extropian-geometry gizmo
  meshes, rendered as unlit overlay parts — `ecosystem.md` §1) for viewport
  manipulation of probes/slices and manual design tweaks; extend it (or wrap
  it) for per-object handle registration, and add drag-commit → undoable
  simulator commands around it. Gizmo editing is 3D-only; it does not live in
  spatial-ui (whose 2D gizmo stack was removed).

Required-spatial-ui-work-tracking: none blocking for v1 workbench; the only
hard dependency is on render's §3 items (which spatial-ui's own UI rendering
also benefits from). Where spatial-ui features are missing (clipping, scroll,
focus wiring), v1 avoids them (panels sized to content; plot panels scrolled
via dedicated range controls) or contributes them upstream as proven.

## 5. Solver Ownership Change

`extropian-solver-fluidx3d` is removed from this repo's dependency graph
(CMake + build.sh done). Consequences:

- No OpenCL requirement; compute is OpenGL-compute within extropian-render.
- The LBM core is written from scratch in `src/physics/lbm/` (GLSL kernels +
  host logic), inheriting lessons from the old wrapper: steady-state
  suspension over frames, amortized readback cadence (~every 500 steps),
  stability monitor with rollback, async regrid on parameter change.
- `extropian-physics` stays linked for its material database and interface
  patterns, but no solver plugin loading is required.

## 6. Contract Review Cadence

- `ecosystem.md` is versioned; render/spatial-ui additions in §3–§4 are the
  agreed work items for `plan.md` Phase 1.
- Each item lands in its own repo with tests; the simulator's `compute/`
  layer CI consumes the upstream `main` branches via FetchContent (local
  checkouts override via `build.sh`), so contract drift surfaces on the
  simulator's own CI immediately.