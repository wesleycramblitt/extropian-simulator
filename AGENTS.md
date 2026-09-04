# AGENTS.md — extropian-simulator

Guide for AI coding agents and human contributors working in this repository.
**These standards are binding**: contributions that violate them will be
rejected.

## What this repo is

The **`exd::sim` library** (ECS-based simulation systems) and the **product
direction** of this repository: an *interactive simulation workspace* —
direct, real-time manipulation of a parametric design in a 3D view where
geometry edits flow into running simulations and results stream back into a
spatial (in-world) dashboard, without leaving the scene.

**Current state:** the workspace UI/UX is the goal; its two load-bearing
ecosystem pieces are not ready yet (`extropian-spatial-ui` document/layout
pipeline and the interactive `extropian-geometry` editing path). Until they
mature, this repo ships exactly two things:

1. the `exd::sim` simulation library (`include/exd/sim/`, `src/`), and
2. headless integration tests (`EXT_SIM_BUILD_TESTS`, on by default, OFF
   when this repo is consumed as a FetchContent dependency).

The demo executables that used to live here (registration examples for
composing sim systems into apps) **moved to `extropian-playground`**
(`playground/demos/`, built on the `DemoApp` host shell at
`playground/demos/common/`). New registration examples and ecosystem
experiments belong there, not here.

```
include/exd/sim/           public library headers (exd::sim namespace)
include/exd/sim/components/  ECS components (plain data structs)
src/                       exd::sim implementation (.cpp files) + headless tests
```

**No assets live in this repo.** Media (cubemaps, meshes, fonts, HDRIs, …)
lives in `extropian-assets` and is fetched from GitHub at configure time
(`FetchContent`). Never add asset directories here.

Build: `./build.sh` → `build/libexd-sim.a` + test binaries in `build/`:
`optimization_test`, `shape_workshop_test`, `solver_run_test`,
`engine_run_test`, `dashboard_feed_test`.

## Ecosystem ownership (where code belongs)

This repo is application-layer *simulation systems only*. Use the other
extropian repos for their purposes and do not reimplement them here:

| Concern | Repo |
|---|---|
| Rendering, graphics context, render systems, components, ImGui host; camera control (`CameraModeSystem` + `CameraModeController`: FPS/orbit/walk/ortho-2D) and the 3D scene tooling path (`PickerSystem`, `SelectionSystem` — gizmo3d is currently being reworked in extropian-render) | `extropian-render` |
| Assets / media (cubemaps, meshes, fonts, hdri, …) | `extropian-assets` |
| Physics solvers, solver plugin interface, fields/BCs (`exd::engine::…` — note: the public API is `exd/engine/…` headers, NOT the legacy `exd/physics/…` paths) | `extropian-physics` |
| Geometry: turbine blades, hubs, primitives, mesh ops, and the gizmo MESH generators (translate/rotate/scale + deform families in `gizmos.hpp` — the single gizmo geometry source). Machines are exposed as `exd::geometry::Assembly` of named, patched `Part`s (the BC contract for solver systems); consume `generate_turbine_assembly()` / `flatten()` | `extropian-geometry` |
| Optimization: CMA-ES, NSGA-II, Nelder-Mead, … | `extropian-optimization` |
| Visualization: field data, colormaps, slices, iso-surfaces, streamlines, particles | `extropian-viz` |
| Spatial UI: layout engine, widget/chart mesh generators, VisualDocument → ECS pipeline (dashboards) — the future workspace UI layer | `extropian-spatial-ui` |
| ECS core, math, config, window state | `extropian-core` |
| Application shell (SDL3/OpenGL window + loop) | `extropian-app` |
| **Demos + ecosystem experiments (incl. the migrated exd::sim demos)** | **`extropian-playground`** |

The exd libraries (except `extropian-assets`; including `extropian-viz`) are
sibling repos fetched via FetchContent; `build.sh` overrides them with
**local checkouts** when present under `../`. `extropian-assets` is the
exception: it is always fetched from GitHub (content-only repo, no
local-sibling override).

## ECS standards (MUST follow)

1. **Components are data, not logic.** Plain structs that satisfy the
   `exd::ecs::Component` concept: trivially movable, trivially destructible,
   no owning pointers, no `std::string`/`std::vector` members. If a component
   needs heap data, it must be a *document component* on a slow entity — see
   `docs/architecture.md` §1.1 — not a hot-path component (this is not currently
   used; do not add one without updating this guide).

2. **Systems are logic, not data.** Systems mutate only the registry. All
   inter-system communication goes through **components on shared entities** —
   never raw pointers to other systems, never getters like
   `mutable_params()`, never `set_other_system()` wiring. The ONLY way to drive
   another system is: find its entity by component (`registry.view<T>()`,
   first match wins) and write the component. System-local ephemeral state
   (dirty snapshots, spin accumulators, optimizer handles) is fine as private
   members — it just is not a cross-system API.

3. **Own your entities.** A system creates its entities lazily in
   `ensure_entities(registry)` on the first `update()` (see `TurbineSystem`
   for the canonical pattern). Panels are their own entities carrying
   `render::ImGuiPanelComponent` — the ImGui host draws one window per such
   entity automatically. Entity names must be unique and descriptive
   (`"Turbine"`, `"TurbinePanel"`, `"OptimizationStudy"`, `"OptimizationPanel"`).

4. **Registry is single-threaded.** It belongs to the app (main thread).
   Structural mutation (create/destroy/emplace/remove) during a `view().each()`
   iteration goes through `ecs::CommandBuffer` — never inline.

5. **Keep system-local state private.** What changes to a component enter the
   registry; what changes to a system (e.g. `running_`, `spin_deg_`) stay
   private members.

## SystemGraph usage (MUST)

- All systems run through `exd::ecs::SystemGraph` phases:
  `Input` (CameraModeSystem) → `Simulation` (sim systems;
  **insertion order = execution order**) → `RenderPreparation`
  (cubemap/polygon/primitive) → `Render` (RenderSystem, then ImGuiSystem).
- Sim systems register in `SystemPhase::Simulation`. Within that phase, order
  is load-bearing: e.g. `OptimizationSystem` must be added *before*
  `TurbineSystem` so it writes `TurbineSpec` before the turbine consumes it.
- The render pipeline is owned by the host shell — do not reorder it.
  Scene tooling (PickerSystem / SelectionSystem / Gizmo3DSystem / GridSystem)
  is registered by the demos as adapters; pointer glue lives in the demo's
  `on_update`, following the extropian-render demo pattern. The registration
  examples now live in `extropian-playground/playground/demos/` (host shell:
  `playground/demos/common/`).
- The physics demos also register the spatial-ui dashboard pipeline in the
  same hook (composer pattern): `scene_renderer` Font/Size/Layout/ViewportFit
  (Structural/Layout) → Mesh/Relation/RenderOrder/ScreenWidget/Camera
  (RenderPreparation) → `render::UIRenderSystem` (Render). The
  `DashboardFeedSystem` (exd::sim) feeds domain components into the
  document nodes; demos stay thin registration layers.

## Component ownership map (current)

| Component | Lives on | Written by | Read by |
|---|---|---|---|
| `TurbineSpec` | `"Turbine"` | TurbineSystem panel, OptimizationSystem | TurbineSystem |
| `OptimizationConfig` | `"OptimizationStudy"` | OptimizationSystem panel | OptimizationSystem |
| `FitnessRecord` | `"OptimizationStudy"` | OptimizationSystem | panels, dashboards |
| `SolverRunConfig` | `"SolverRun"` | SolverRunSystem panel | SolverRunSystem |
| `SolveRunState` | `"SolverRun"` | SolverRunSystem | panels, dashboards |
| `EngineSpec` | `"SteamEngine"` | SteamEngineSystem panel, OptimizationSystem (engine mode) | SteamEngineSystem |
| `EngineRunState` | `"SteamEngine"` | SteamEngineSystem | panels, dashboards |
| `IndicatorRecord` | `"SteamEngine"` | SteamEngineSystem | panels, spatial-ui dashboards |
| `ShapeWorkshopSpec` | each `"Shape.N"` | ShapeWorkshopSystem panel | ShapeWorkshopSystem |
| `CameraModeController` | `"Camera"` | demo panels / hotkeys | CameraModeSystem |
| `GizmoModeComponent` | `"Tools"` | demo hotkeys / panel | Gizmo3DSystem |

## Background-work threading contract

Background runs (`SolverRunSystem`, the coupled-CFD objective mode of
`OptimizationSystem`, and `SteamEngineSystem`) execute their physics calls
(`run_coupled_turbine()`, `simulate_engine()`) on a worker thread
(`std::async`, one job at a time). The worker touches **no registry, no
render, no ImGui** — it returns a heap payload through a `std::future`, and
the main thread adopts it and writes components / uploads meshes. Poll with
`future_.wait_for(0)`; never join-block in the frame loop.

## Objective models (OptimizationSystem)

`OptimizationSystem` is constructed with an `ObjectiveModel`:
`Analytic` (inline fast objective — the default demo), `CoupledCfd`
(one short 12³ FDM3 coupled run per candidate, evaluated sequentially on a
background worker, ~0.7 s each) or `EngineSim` (inline 0D steam-engine
simulation, ~ms each). Candidate designs are always mapped through the
shared recipes in `src/coupled_run.hpp` / `src/engine_run.hpp`
(single source of truth, mirrored by `solver_run_test` / `engine_run_test`).

## How to add a system (the researcher workflow this repo serves)

1. Add component(s) to `include/exd/sim/components/` (POD, documented
   writers/readers/entity).
2. Add `include/exd/sim/<name>_system.hpp` + `src/<name>_system.cpp`:
   `ISystem` subclass, `ensure_entities` pattern, inputs via views, outputs
   via registry writes, optional self-registered panel.
3. Add the `.cpp` to the `exd-sim` target in `CMakeLists.txt`.
4. Add a registration example in `extropian-playground/playground/demos/`
   (new directory + `main.cpp` subclassing `DemoApp`, registering the
   system; see `playground/demos/optimization` for the minimal shape), and
   add the executable + asset-copy entry in
   `playground/demos/CMakeLists.txt`.
5. Update the component ownership map above.

## Conventions

- Namespace `exd::sim`; headers under `include/exd/sim/`; consumers include
  `<exd/sim/...>` only (library boundary).
- C++23. POD structs for components with default-member initializers.
- `std::printf` lifecycle diagnostics (`[Turbine]`, `[Optimization]`,
  `[DemoApp]`) — consistent with the existing codebase.
- Pure physics/numerics go in anonymous namespaces inside `.cpp` files so they
  stay testable without ECS/GPU (see `src/optimization_system.cpp`).
- **Physics API note:** `extropian-physics` exposes its headers under
  `exd/engine/...` (`exd::engine::presets::engine`, `exd::engine::presets::turbine`,
  `exd::engine::core::ModelStatus`, ...). The legacy `exd/physics/...` paths
  are gone — do not reintroduce them.

## Testing

- Headless tests in `src/*_test.cpp`, built when `EXT_SIM_BUILD_TESTS` is ON
  (the default; OFF when consumed as a dependency).
- `optimization_test` reproduces the objective standalone (links
  `exd::optimization` only). Keep pure logic extracted from systems so it
  can be tested headlessly.
- After changes: `./build.sh`, then run the headless tests
  (`./build/optimization_test`, `shape_workshop_test`, `solver_run_test`,
  `engine_run_test`, `dashboard_feed_test`).

## Touch rules

- **Do not** reintroduce cross-system raw pointers or setter-based wiring —
  including in demo code, which is the documented teaching surface.
- **Do not** move sim systems into the host shell or into an app executable;
  `exd::sim` is the library, demos are thin registration layers in
  `extropian-playground`.
- **Do not** add windowed demos here — they belong in `extropian-playground`;
  this repo ships library + headless tests, and future UI/UX work toward the
  interactive workspace.
- When a change touches this guide, update it in the same commit.
