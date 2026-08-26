# Extropian Simulator — Architecture and Implementation Plan

> **A real-time multiphysics inverse design and optimization engine.**
> Type a prompt like *"Design a 3 MW wind turbine with low noise, 60 m blades,
> class IB loads"* — the system produces a parametric CAD model (or imports
> yours), auto-configures fitness functions, meshing, boundary conditions and
> solvers, runs aggressive optimization loops with custom fast solvers, and
> finishes with a live, interactive post-optimization run in which wind speed,
> turbulence, loading and rotor speed can be changed in real time.

The old plan described a conventional CAE workbench (geometry → mesh → solve →
postprocess) with Setup/Simulate/Postprocess modes. That direction is
superseded. This document replaces it.

Companion documents (all under `docs/`):

| Document | Scope |
|---|---|
| `plan.md` (this file) | Vision, product workflow, architecture overview, roadmap |
| `architecture.md` | ECS component/system inventory, threading, GPU compute layer, repo layout |
| `solvers.md` | Physics stack: BEMT, GPU LBM, structural beam, turbulence, accuracy tiers |
| `optimization.md` | Parameterization, fitness, algorithms, multi-fidelity, surrogates |
| `postprocessing.md` | Live-run loop, real-time visualization, export/verification package |
| `ecosystem.md` | What sibling Extropian repos provide today and what must be added |

## 1. Vision

Extropian Simulator is a **design engine**, not a passive analysis tool. The
user states a design goal; the engine owns everything needed to converge on an
optimized physical product:

1. **Specification intake** — a natural-language prompt (LLM-assisted) or a
   structured spec is parsed into a `DesignSpec` with engineering quantities.
2. **Geometry synthesis** — parametric CAD is generated from the spec (airfoils,
   blade build-up, hub, nacelle, tower) or imported from the user's CAD.
3. **Auto-configuration** — a rules engine derives the study from the spec:
   fitness functions, design variables, meshing parameters, boundary
   conditions, solver selection and fidelity tier. No manual setup.
4. **Aggressive optimization** — thousands of candidate evaluations per second
   using tiered solvers (instant 1-D blade momentum theory, GPU lattice
   Boltzmann, structural beams), surrogate models and reduced-order models.
5. **Live post-optimization run** — the winner is simulated in real time with
   live-adjustable environment (wind speed, turbulence, yaw, rotor speed) and
   full visualization (streamlines, slices, iso-surfaces, probes, force/power
   plots).
6. **Verification export** — geometry (STL/STEP), fields (VTK/CSV) and a study
   report that import cleanly into high-fidelity commercial software for
   final verification and fine tuning.

Design principles:

- **Everything is ECS.** Design state, solver state, results, live settings and
  visualization state are entities and components. Systems transform them.
- **GPU-first computation.** Solvers are custom-built, compute-shader based,
  and designed for real-time throughput. CPU parallelism where GPU is
  inappropriate (1-D methods).
- **Accuracy is tiered, not uniform.** The engine targets ~80–90% engineering
  accuracy for its fastest tiers (less in early optimization), converging to
  full "hand over to real CFD" fidelity only for the final candidate. Speed is
  a correctness requirement: the product is the *loop*, not the single solve.
- **No external solver dependencies.** FluidX3D is dropped. All solvers live in
  this repository (`src/physics/`). The only external GPU dependency is the
  OpenGL 4.3+ compute pipeline already loadable through `extropian-render`.
- **UI comes from extropian-spatial-ui**, rendering through extropian-render.
  Post-processing logic is owned by this repository.
- **The engine generalizes.** Wind turbines are the first application; the
  solver/optimization framework is domain-independent.

## 2. Product Workflow

```text
┌──────────┐   ┌──────────────┐   ┌──────────────────┐   ┌─────────────┐
│  Prompt   │──▶│  DesignSpec  │──▶│  AutoConfig       │──▶│  Study       │
│  (LLM+    │   │  (typed,     │   │  (rules: fitness, │   │  (variables, │
│  rules)   │   │  validated)  │   │   mesh, BCs,      │   │  objectives, │
└──────────┘   └──────────────┘   │   solvers, tier)  │   │  constraints)│
                                  └──────────────────┘   └──────┬──────┘
        ┌───────────────┐    ┌───────────────┐    ┌─────────────▼──────┐
        │  CAD from      │    │  CAD import   │    │  Optimization loop  │
        │  parametric    │    │  (STL/OBJ…    │    │  candidate x        │
        │  generator     │    │   assimp)     │    │  BEMT / LBM /       │
        └───────┬───────┘    └───────┬───────┘    │  structural /        │
                └──────────┬────────┘             │  surrogate           │
                           ▼                      └──────────┬──────────┘
                  Geometry/model                              ▼
                  (ECS entity, parametric)      Winner: live real-time run
                                               wind speed / turbulence /
                                               TSR adjustable live
                                               streamlines, slices, probes
                                                         ▼
                                  Export package: STL/STEP + VTK/CSV
                                  + study report → high-fidelity CAD/CFD
```

## 3. Application Modes

The old Setup/Simulate/Postprocess modes are replaced by four workflow modes:

| Mode | Purpose |
|---|---|
| **Design** | Prompt/spec intake, review of auto-configuration, geometry generation or CAD import, study acceptance |
| **Optimize** | Run optimization loop; live convergence, fitness histograms, best-candidate inspection; pause/resume/seed |
| **Live Run** | Real-time post-optimization simulation with live-adjustable environment and visualization |
| **Export** | Verification package: STL/STEP geometry, VTK/CSV fields, report; sanity checks |

Modes are implemented with the existing `exd::app` mode machinery
(`modes().define(...)`, `on_mode_changed`), and every mode shares the same
registry — switching modes only changes which systems are enabled and which UI
panels are visible.

## 4. Architecture Overview

### 4.1 Layers

```text
┌──────────────────────────────────────────────────────────────────────┐
│  app/            exd::app Application: window, modes, frame loop      │
├──────────────────────────────────────────────────────────────────────┤
│  ui/             spatial-ui workbench: prompt bar, monitors,          │
│                  inspectors, plots (VisualDocument + interaction)     │
├──────────────────────────────────────────────────────────────────────┤
│  ecs/            domain ECS: components + systems (this repo)         │
│  ┌────────────┬────────────┬────────────┬──────────────┬───────────┐ │
│  │ design     │ meshing    │ solvers    │ optimization │ postprocess│ │
│  │ systems    │ systems    │ (ECS wrap) │ systems      │ systems    │ │
│  └────────────┴────────────┴────────────┴──────────────┴───────────┘ │
├──────────────────────────────────────────────────────────────────────┤
│  physics/      solver cores (no ECS): BEM, LBM (GLSL kernels),        │
│                structural beam, turbulence generation, thread pool    │
├──────────────────────────────────────────────────────────────────────┤
│  compute/      GPU buffer/pipeline wrappers (SSBO, ping-pong, 3D tex) │
├──────────────────────────────────────────────────────────────────────┤
│  extropian-render   render backend: passes, techniques, GPU compute   │
│  extropian-core     ECS registry, math, JSON, serialization           │
│  extropian-spatial-ui  UI widgets, layout, interaction, gizmos        │
└──────────────────────────────────────────────────────────────────────┘
```

The split between `physics/` (pure solver cores) and `ecs/systems/` is the
critical one: **solver cores are thread-agnostic numeric engines with explicit
data-in/data-out; ECS systems are thin adapters** that move data between the
registry and the cores. This keeps the (thread-unsafe) registry on one thread
while numeric work parallelizes freely.

### 4.2 System graph

Systems register with the application system graph (`graph.add<T>(...).always()`,
`.in_mode(...)`, plus explicit `order_before` where needed). Conceptually the
graph groups into ordered stages:

```text
Input → Structural → Layout → Interaction → Simulation → RenderPreparation → Render
```

- **Input**: prompt/intake, CAD import requests.
- **Simulation**: solvers, optimization, post-process derived fields. This
  stage holds the heavy compute; systems here communicate with GPU cores via
  compute/ wrappers and with CPU cores via thread pools.
- **RenderPreparation**: conversion of result components into render
  components (mesh uploads, particle buffers, volume textures, overlays).
- **Render**: the existing `exd::render::RenderSystem` passes.

`exd::core`'s `SystemGraph` defines a phase enum plus order; the app-level
graph used here supports registration-order + `order_before` dependencies. The
simulator introduces the stage grouping as a documented convention (see
`architecture.md` §3), and pushes the phase enum up to `exd::app`/`exd::core`
once the convention proves out.

### 4.3 Threading and real-time model

- The ECS registry, UI and renderer live on the **main thread** (single frame
  loop, ~16 ms/frame budget at 60 Hz).
- **GPU solver cores** are dispatched synchronously from the Simulation phase;
  GL compute dispatch is non-blocking, and steady-state LBM converges in
  seconds at coarse resolutions (see `solvers.md` budgets).
- **CPU solver cores** (BEMT) run on an internal thread pool owned by the core;
  results are published through double-buffered slots with atomic flags so the
  main thread only ever reads committed snapshots.
- **Optimizer loops** never block the frame: each frame the optimizer system
  performs a bounded batch of candidate evaluations (µs-scale BEMT evals) and
  writes progress components. Long-acquire operations (LLM calls, large STL
  import) run on a background task queue and post completion commands.
- Full detail in `architecture.md` §4.

### 4.4 GPU compute ownership

`extropian-render` owns all OpenGL state. The simulator requires render to
expose the compute substrate (compute-shader pipelines, SSBO buffers, 3D
texture upload, headless rendering); the simulator's `compute/` layer is a thin
typed wrapper over those facilities and owns nothing GL-raw. Required render
additions are enumerated in `ecosystem.md` §3 — they are all already loaded in
the vendored GLAD 4.6 set (`glDispatchCompute`, SSBO, `glMapBufferRange`, 3D
textures).

## 5. ECS Domain Model (summary)

Full component/system catalog with fields: `architecture.md` §2–§3.

Top-level domain entities:

```text
DesignSpec          — typed, validated engineering intent from prompt parsing
ParametricModel     — geometry parameters (blades, hub, nacelle, tower)
GeometryEntity      — generated or imported CAD surface (renderable, voxelizable)
StudyDefinition     — design variables, objectives, constraints, solver config
VoxelGrid           — LBM lattice + GPU field handles
SolverComponent     — (tag) points at a solver core instance + config
SolverStatus/Stats  — running/converged/diverged, step, residual, forces
OptimizerState      — algorithm, generation, population, best-so-far
FitnessRecord       — per-candidate objective/constraint results (catalog)
InflowConfig        — wind speed, turbulence intensity, shear profile, yaw
TurbineControl      — rotor speed/TSR, pitch offset, brake
Probe / SlicePlane / IsoSurfaceSpec / StreamlineSpec — post-process queries
DerivedFieldSpec    — vorticity, Cp/Ct extraction, wake deficit, etc.
ExportRequest       — geometry/field/report export jobs
```

Component hygiene follows `exd::ecs::Component` rules: POD, trivially movable,
no owning pointers. GPU resources are referenced by opaque handles
(`uint32_t` texture/buffer ids), never by raw GL objects inside components.

## 6. Solver Stack (wind turbine v1)

Four custom solver cores, tiered by accuracy/cost (detailed in `solvers.md`):

| Tier | Solver | Eval cost | Accuracy | Used in |
|---|---|---|---|---|
| T0 | **BEMT** (blade element momentum + corrections), CPU thread-pool | 10–100 µs | 70–85% power, 80%+ loads | Early optimization loops, surrogate training data |
| T1 | **ALM-LBM GPU** (actuator-line rotor + D3Q19 lattice, Smagorinsky), 96³–160³ | ~1–5 ms/step; settle 2–10 s (warm-started) | 80–90% wake; loads ±20–40% | Mid/late loops, **live run mode** (transient) |
| T2 | **Blade-resolved LBM** (local refinement, effective 192³–320³) | 10–60 s/settle | 90%+ (Cp ±5–10%) | Final candidate verification inside engine |
| T3 | Export to external high-fidelity CFD/FEA | — | verification | Outside the engine |

Throughput assumption: fused D3Q19 kernels at ~1 GLUPS on a mid-range desktop
GPU (benchmark target `benchmarks/mlups`); all settle times assume
warm-started fields from lower tiers (`optimization.md` §5.3).

Supporting cores:

- **Structural beam** (Timoshenko, per-blade) — deflection and stress
  constraints from T0/T1 loads; aero-elastic double loop when enabled.
- **Turbulence inflow** — synthetic spectral turbulence (veers/Mann-style)
  parameterized by turbulence intensity and scale; regenerates live on
  adjustment, feeds the LBM inlet.
- **Airfoil database** — NACA 4/5-digit parametric shapes (analytical
  polars via XFOIL-computed tables shipped in `assets/`) plus catalog airfoils
  (chord/twist/thickness distributions).

## 7. Optimization Engine (summary)

Detailed in `optimization.md`.

- **Design variables**: continuous (rotor radius, twist/chord B-spline control
  points, cone/tilt, tower height, airfoil camber/thickness params) and
  discrete (blade count, airfoil family selection).
- **Fitness**: weighted objectives from `DesignSpec` (AEP via power curve ×
  Weibull wind distribution, thrust, loads, noise proxy) with hard constraints
  (max stress, deflection, tip clearance, class-IB load envelope).
- **Algorithms**: CMA-ES (primary, continuous), NSGA-II (multi-objective),
  restartable pattern search; all population-based so evaluations batch onto
  the GPU/thread pool.
- **Multi-fidelity scheduling**: early generations evaluate entire populations
  at T0 (thousands per second); promising elite subsets promote to T1, then
  T2 for the finalists. Fidelity assignment is part of the study definition
  and adaptive (promotion thresholded by surrogate uncertainty).
- **Surrogates/ROMs**: Gaussian-process fitness surrogate fitted to evaluated
  candidates (correcting T0 errors against T1 samples); snapshot PCA of LBM
  fields for cheap wake-feature estimation and warm-starting.
- **Aggression targets**: 5 000–50 000 T0 evals per optimization run
  (~5–60 s wall, frame-bounded batches), 30–300 T1 evals (minutes), ≤ 10 T2
  evals (finalists, `optimization.md` §7).

## 8. Real-Time Post-Processing and Live Run

Detailed in `postprocessing.md`. This repository owns post-processing logic.

- Live-adjustable inputs: wind speed (0–60 m/s), turbulence intensity
  (0–30%), shear exponent, yaw angle, rotor speed (TSR), blade pitch offset.
- Change semantics: integrated quantities (power curve, loads) update
  instantly via T0/BEMT; the T1 lattice receives new inlet BCs/turbulence
  layers and actuator-line forces, and the wake responds *visibly* within
  ~1–3 s (convective timescale), settling fully in tens of seconds at
  utility scale. The "settled" badge distinguishes transient from converged.
  The rotor is an actuator-line model at T1 — speed changes need no regrid.
- Visualization: GPU particle advection (pathlines/streamlines), velocity/
  pressure slices (3D-texture sampling), iso-surfaces (GPU marching cubes or
  compute raymarching), surface contours, probes with time histories, power
  and force live plots, rotor animation.
- Probe/plot data flows through spatial-ui descriptor meshes
  (`generate_bar_mesh`, `generate_scatter_mesh`, `generate_sparkline_mesh`,
  etc.) into the render queue; number-heavy panels (variable tables, fitness
  histograms) use `exd::ui` list/table mesh generators.

## 9. Accuracy and Performance Contracts

- **Fidelity is a first-class setting.** Every solver states its accuracy
  envelope; the optimizer never misattributes T0 error as trend.
- Frame budget (60 Hz): all non-solver systems < 2 ms; T1 LBM 1–4 steps/frame
  (~1–8 ms GPU) in live mode; optimizer batches bounded ≤ 8 ms main-thread
  time per frame (practically 500–2 000 T0 evals/frame fed to the worker
  pool — the pool does the heavy lifting off-thread).
- Numerical safety: NaN/Inf guards after every solver step (stability monitor
  that flags `SolverStatus::Error` and can roll back a run); CFL-based
  adaptive stepping in LBM; BEMT convergence failure → candidate penalized,
  not crashed.
- Determinism: seeded PRNG (core's PCG32) for optimizer and turbulence so
  runs are reproducible in CI.

## 10. Repository Layout

```text
src/
  main.cpp                        — app bootstrap, modes
  app/                            — application wiring, frame loop, workbench host
  ecs/
    components/                   — design, study, mesh, solver, optimization,
    │                               live, postprocess component headers
    systems/
      design/                     — prompt_intake, auto_config, parametric_geometry,
      │                             cad_import
      meshing/                    — voxelization
      solvers/                    — bem_wrap, lbm_wrap, structural_wrap, inflow
      optimization/               — optimizer, surrogate, fidelity_scheduler
      postprocess/                — streamlines, slices, isosurfaces, probes,
      │                             derived_fields, force_report
      live/                       — live_controller, turbine_control
      export/                     — cad_export, field_export, study_report
  compute/                        — gpu_buffer, compute_pipeline, ping_pong
  math/                           — splines, polars, interpolation extras
  physics/
    bem/                          — BEMT core (CPU, thread pool)
    lbm/                          — LBM core + GLSL kernels + host logic
    structure/                    — Timoshenko beam core
    turbulence/                   — synthetic inflow core
  ai/                             — spec_parser (rules) + LLM adapter interface
  io/                             — airfoil_db, cad import/export, field export
  ui/                             — view models bridging ECS → spatial-ui
assets/
  airfoils/                       — polar tables, parametric definitions
tests/  benchmarks/               — see §12 and architecture.md §7
```

## 11. Roadmap

### Phase 0 — Foundations and contracts
- Remove remaining prototype-era scaffolding; restructure into the layout
  above (FluidX3D/solver-wrapper references already removed).
- Add `extropian-spatial-ui` to the CMake dependency graph (FetchContent,
  mirroring the other repos; `build.sh` local-override loop already includes
  it).
- Define ECS component skeletons: `DesignSpec`, `ParametricModel`,
  `StudyDefinition`, `OptimizerState`, solver components, and the canonical
  evaluation contract (`SolveResult → FitnessRecord`, `architecture.md` §5.5).
- Document the exact interface contracts required from extropian-render
  (compute substrate) and spatial-ui (workbench) — `ecosystem.md` is the
  living contract.
- CI baseline: clean build, unit tests green, headless smoke test
  (`IRenderer::create(Backend::Null)` fix in render, `ecosystem.md` §3).

**Exit**: repo builds with new layout; contracts reviewed and accepted by the
render and spatial-ui repos.

### Phase 1 — Compute substrate (render repo + simulator glue)
- extropian-render: compute-shader compile path, `GpuBuffer`/SSBO with
  persistent mapping, 3D `ITextureSource` + `TextureManager::update` path,
  FBO `RenderTarget`, headless `NullRenderer` wiring (`ecosystem.md` §3).
- Simulator `compute/` layer: typed buffers, ping-pong, compute pipelines,
  frame-budget instrumentation.

**Exit**: a compute-shader Poisson solve runs, round-trips through
`TextureManager::update`, and is visualized as a 3D scalar field.

### Phase 2 — Geometry and meshing
- Airfoil database + NACA parametric generation.
- Parametric rotor/hub/nacelle/tower generator → renderable `GeometryEntity`
  + B-spline variable definitions.
- CAD import (assimp already wired in the prototype) normalized into
  `GeometryEntity`.
- GPU-friendly voxelizer for LBM (octree coarse-to-fine, watertight check).

**Exit**: a spec-driven turbine model and its voxel lattice appear in the
workbench; imported STL voxelizes correctly.

### Phase 3 — BEMT solver and optimization v1
- BEMT core (steady, Prandtl/Glauert corrections, skew, yaw) with thread
  pool; `BEMTSolverSystem`.
- Optimizer core: CMA-ES + NSGA-II; `OptimizerSystem`, `FitnessRecord`
  catalog; deterministic seeding.
- `AutoConfigSystem` (rules) mapping `DesignSpec` → `StudyDefinition`;
  `SpecParser` (rule-based) v1.
- Fitness: AEP objective, thrust/loads/stress constraints; power curve
  computed by BEMT over the wind speed grid.

**Exit**: "Design a 2 MW turbine…" converges to a plausible design in
seconds with power curve, loads and constraint report; reproduces the known
NREL 5 MW design when constrained to its variable set (golden test).

### Phase 4 — GPU LBM solver (T1 actuator-line, then T2 blade-resolved)
- D3Q19 compute-shader core: collision (BGK/TRT), streaming, Smagorinsky
  LES-lite, inlet/outlet/free-slip BCs, synthetic turbulence inlet layer.
- **T1 actuator line**: BEMT section loads (resolved local inflow) projected
  as body forces onto the lattice; explicit rotor rotation from
  `TurbineControl`; live parameter rewrite (`InflowConfig`,
  `TurbineControl` → BC/actuator update without solver restart).
- Stability monitor, CFL guard, restart-from-snapshot; MLUPS benchmark vs the
  1 GLUPS target.
- **T2 (second milestone)**: local refinement (2–3 levels) + rotating blade
  geometry voxels, curved-wall BCs, momentum-exchange blade forces.
- Validation: Taylor–Green, Poiseuille, lid-driven cavity, then ALM wake +
  forces vs wind-tunnel cases and the NREL 5 MW golden operating point.

**Exit**: live T1 run at 96³–160³; changing wind speed or turbulence
intensity visibly changes the wake within ~1–3 s and settles in seconds-to-
tens-of-seconds; T2 resolves one golden operating point within its envelope.

### Phase 5 — Post-processing and Live Run UX
- Post-process systems (streamlines, slices, iso-surfaces, probes, derived
  fields) writing render-ready components (`postprocessing.md`).
- Live workbench: sliders (wind, TI, shear, yaw, TSR), power/force plots,
  rotor animation, wake visualization.
- Workbench v1 on spatial-ui: prompt bar, study tree, variable inspector,
  fitness panels, plots.

**Exit**: full-mode demo — prompt → optimize → live run with adjustable
environment and rich visualization at interactive rates.

### Phase 6 — Structural, multiphysics, surrogates, export
- Timoshenko beam core + aero-elastic coupling loop.
- Surrogate (GP) + multi-fidelity scheduler activation: budgets shift T0→T1
  adaptively.
- Snapshot-PCA ROM of LBM fields for warm-starts and wake-feature fitness.
- Export systems: STL/STEP geometry, VTK/CSV fields, study report JSON.

**Exit**: optimized design passes structural constraints; export package
opens in external CAD/CFD (golden round-trip test); optimizer uses surrogates
to cut T1 budget ≥ 50% at equal final fitness.

### Phase 7 — Prompt intelligence and productization
- LLM spec-parser adapter (provider-agnostic interface; config-driven like
  the composer's `ai.*` sections). LLM maps prose → `DesignSpec`; the rules
  engine remains authoritative for engineering decisions.
- Workbench polish: saved studies, comparison of candidate designs, undo/
  redo through spatial-ui commands, gizmo-based manual tweaks (extend
  `exd::render::GizmoSystem` or spatial-ui gizmo module).

**Exit**: free-form prompts ("quiet 2 MW turbine for a ridge site with
class-II winds") produce valid, converged studies with no manual setup.

### Phase 8 — Generalization to other physics
- Register additional solver cores behind the same system/component
  contracts (heat exchangers, ducts, acoustics, multibody mechanisms).
- Reusable pieces promoted to spatial-ui/render/core only after proven use.

**Exit**: a second domain runs the full prompt → optimize → live loop.

## 12. Testing and Validation Strategy

### Unit tests
- Each solver core against analytic limits: BEMT at zero/ideal loading vs
  Betz; LBM decay tests (Taylor–Green), Poiseuille channel, lid-driven
  cavity; beam core cantilever vs Euler beam analytic.
- Optimizer convergence on known convex/test problems; determinism with
  fixed seeds.
- ECS systems: component invariants, catalog correctness, live BC rewriting.

### Golden tests
- NREL 5 MW constrained re-optimization: power curve and loads within
  published values ± tier accuracy.
- Turbine-on-lattice standard case: forces and wake centerline deficit vs
  experiment/verified CFD references.

### Integration tests
- Prompt → DesignSpec → Study → geometry → voxel → solve → results
  round trip (headless).
- Live-run change semantics: wind speed bump → expected power trend and
  re-convergence time bounds (CI on CPU LBM fallback).
- Export round-trip: exported geometry re-imports cleanly; VTK parses in
  external readers.

### Performance budgets (benchmarks/)
- BEMT evals/s, LBM MLUPS (million lattice updates per second), steady-state
  time-to-converge per tier, live-run frame timings, optimizer generations/s.
- CI guards regressions against recorded baselines.

## 13. Non-Goals

- Full-fidelity CFD/FEA inside the engine — final verification happens in
  external high-fidelity tools via the export package.
- A general CAD kernel — parametric generation + import covers v1; STEP
  export uses tessellated geometry.
- Retained widget toolkit development in this repo — workbench chrome comes
  from spatial-ui; this repo owns view models and workflows only.
- LLM authority over engineering parameters — LLM maps prose to a validated
  `DesignSpec`; the rules engine and solvers decide the rest.
- ECS as a durable project database — study files are JSON documents;
  ECS is runtime state.
- Real-time accuracy at 100% — tiered accuracy is a product feature, not a
  limitation.

## 14. Near-Term Deliverables

1. Phase 0 restructure + component skeletons (this plan's layout).
2. `ecosystem.md` contract review with the render and spatial-ui repos;
   Phase 1 compute substrate items queued in extropian-render.
3. Rule-based `SpecParser` + `AutoConfigSystem` + `StudyDefinition` JSON
   format with fixtures.
4. BEMT core with golden NREL 5 MW test.
5. Parametric turbine generator producing renderable geometry.
6. Workbench shell v1 (prompt bar + monitor panels) on spatial-ui.

These deliverables validate the central boundary: domain state lives in this
repo's ECS and JSON studies, GPU compute flows through extropian-render's
compute substrate, all UI comes from spatial-ui, and the optimization loop —
not a single solve — is the product.