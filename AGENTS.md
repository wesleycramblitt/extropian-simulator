# AGENTS.md — extropian-simulator

Guide for AI coding agents and human contributors working in this repository.
**These standards are binding**: contributions that violate them will be rejected.

## What this repo is

The **`exd::sim` library** (ECS-based simulation systems) plus **demos** that show
researchers how to compose systems into applications. The product model: researchers
build their own apps against this library, registering systems into an ECS
`SystemGraph` — "plug and play" by manipulating the ECS. The demo executables are
the copyable templates for that. Longer-term product vision: `docs/architecture.md`.

```
include/exd/sim/           public library headers (exd::sim namespace)
include/exd/sim/components/  ECS components (plain data structs)
src/                       exd::sim implementation (.cpp files)
demos/common/              DemoApp host shell (window + render pipeline + ImGui)
demos/<name>/              one demo executable per registration example
```

**No assets live in this repo.** Media (cubemaps, meshes, fonts, HDRIs, …) lives in
`extropian-assets` and is fetched from GitHub at configure time
(`FetchContent`), then copied next to each demo binary at build time.
Never add asset directories here — add them to `extropian-assets` and to the
`EXD_SIM_DEMO_ASSET_DIRS` list in `demos/CMakeLists.txt`.

Build: `./build.sh` → binaries in `build/`:
- `build/extropian-sim-shape-workshop`  — DEFAULT demo: parametric primitive
  gallery (2D + 3D shapes from extropian-geometry recipes) with live
  parameter control and 3D gizmo editing (the minimal copyable template)
- `build/extropian-sim-camera-modes`    — unified mobile camera: FPS / orbit /
  ground-walk / orthographic-2D editing + runtime FOV
- `build/extropian-sim-optimize`        — analytic-objective optimization
  reference
- `build/extropian-sim-turbine-solver`  — real meshing + FDM3 solver run +
  viz + coupled-CFD optimization
- `build/extropian-sim-steam-engine`    — steam engine meshing + 0D engine
  solver + optimization + indicator dashboards
- `build/optimization_test`, `build/shape_workshop_test`, `build/solver_run_test`,
  `build/engine_run_test`, `build/dashboard_feed_test` — headless CLI tests

Run: `./run.sh [demo]` (default: shape-workshop). `Z` toggles fly camera / UI
mode. In the shape-workshop demo, `1/2/3` switch the 3D gizmo mode.

## Ecosystem ownership (where code belongs)

This repo is application-layer *simulation systems and demos only*. Use the
other extropian repos for their purposes and do not reimplement them here:

| Concern | Repo |
|---|---|
| Rendering, graphics context, render systems, components, ImGui host; camera control (`CameraModeSystem` + `CameraModeController`: FPS/orbit/walk/ortho-2D) and the 3D scene tooling path (`PickerSystem`, `SelectionSystem`, `Gizmo3DSystem` — 3D gizmo interaction on geometry gizmo meshes) | `extropian-render` |
| Assets / media (cubemaps, meshes, fonts, hdri, …) | `extropian-assets` |
| Physics solvers, solver plugin interface, fields/BCs | `extropian-physics` |
| Geometry: turbine blades, hubs, primitives, mesh ops, and the gizmo MESH generators (translate/rotate/scale + deform families in `gizmos.hpp` — the single gizmo geometry source). Machines are exposed as `exd::geometry::Assembly` of named, patched `Part`s (the BC contract for solver systems); consume `generate_turbine_assembly()` / `flatten()` | `extropian-geometry` |
| Optimization: CMA-ES, NSGA-II, Nelder-Mead, … | `extropian-optimization` |
| Visualization: field data, colormaps, slices, iso-surfaces, streamlines, particles | `extropian-viz` |
| Spatial UI: layout engine, widget/chart mesh generators, VisualDocument → ECS pipeline (dashboards) | `extropian-spatial-ui` |
| ECS core, math, config, window state | `extropian-core` |
| Application shell (SDL3/OpenGL window + loop) | `extropian-app` |

The exd libraries (except `extropian-assets`; including `extropian-viz`) are sibling repos fetched via
FetchContent; `build.sh` overrides them with **local checkouts** when present
under `../`. `extropian-assets` is the exception: it is always fetched from
GitHub (content-only repo, no local-sibling override).

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
- Demo apps register sim systems in `DemoApp::register_sim_systems(graph)`.
  The render pipeline is owned by the host shell — do not reorder it.
  Scene tooling (PickerSystem / SelectionSystem / Gizmo3DSystem / GridSystem)
  is registered by the shape demos as adapters; pointer glue lives in the
  demo's `on_update`, following the extropian-render demo pattern.
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
4. Add a demo in `demos/<name>/` (new directory + `main.cpp` subclassing
   `DemoApp`, registering the system; see `demos/optimization` for the
   minimal shape), and add the executable + asset-copy entry in
   `demos/CMakeLists.txt`.
5. Update the component ownership map above.

## Conventions

- Namespace `exd::sim`; headers under `include/exd/sim/`; consumers include
  `<exd/sim/...>` only (library boundary).
- C++23. POD structs for components with default-member initializers.
- `std::printf` lifecycle diagnostics (`[Turbine]`, `[Optimization]`,
  `[DemoApp]`) — consistent with the existing codebase.
- Pure physics/numerics go in anonymous namespaces inside `.cpp` files so they
  stay testable without ECS/GPU (see `src/optimization_system.cpp`).

## Testing

- `optimization_test` reproduces the objective standalone (links
  `exd::optimization` only). Keep pure logic extracted from systems so it can
  be tested headlessly.
- After changes: `./build.sh`, then run both demos briefly and `./build/optimization_test`.

## Touch rules

- **Do not** reintroduce cross-system raw pointers or setter-based wiring —
  including in demo code, which is the documented teaching surface.
- **Do not** move sim systems into the host shell or into an app executable;
  `exd::sim` is the library, demos are thin registration layers.
- When a change touches this guide, update it in the same commit.
