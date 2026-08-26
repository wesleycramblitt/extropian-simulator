# Architecture — ECS Model, Compute Layer, Concurrency

> Implementation detail for `plan.md` §4–§5. Assumes the ecosystem status in
> `ecosystem.md`. All names below are proposals for `exd::sim` / `exd::sim::sys`
> namespaces; adjust to taste, keep the *boundaries* stable.

## 1. Guiding Rules

1. **Registry is single-threaded.** The `exd::ecs::Registry` (sparse-set,
   generation-guarded) is owned by the main thread. Views iterate the smallest
   pool; structural mutation during iteration goes through `CommandBuffer`.
2. **Solver cores own their parallelism.** A core (`BEMTCore`, `LBMCore`,
   `BeamCore`) is a plain object with `solve(Job) -> JobResult`. It may spawn
   threads or dispatch GPU work internally. The wrapping system copies
   inputs from components, hands them to the core, and publishes results back
   into components.
3. **GPU resources referenced by handle, not by pointer.** Components carry
   `uint32_t` buffer/texture handles owned by `compute/` wrappers, which in
   turn are the only owners of render's GPU objects. Destroy order: wrapper
   teardown at registry `clear()` / app shutdown.
4. **Nothing expensive runs on the main thread more than once per budgeted
   batch.** Optimization, LLM calls, imports and exports are always
   backgrounded or bounded.
5. **Systems are deterministic by default** (seeded RNGs, fixed iteration
   orders) so CI tests and optimization runs reproduce.

### 1.1 Component classes — two tiers, stated honestly

`exd::ecs::Component` formally requires trivially-move-constructible,
trivially-destructible types without owning pointers. The simulator uses two
component classes and says so explicitly (this relaxes the formal rule for
document components; both work with the registry's dense pools in practice,
but the distinction keeps hot paths safe):

- **POD hot-path components**: `InflowConfig`, `TurbineControl`, `VoxelGrid`
  (handle + scalars), `SolverStatus/Stats`, `Transform`-style data, slice/
  probe specs. These are what per-frame views iterate; they satisfy the
  `Component` concept exactly.
- **Document components**: `DesignSpec`, `ParametricModel`,
  `StudyDefinition`, `FitnessRecord`, `ExportRequest` — structs that may own
  `std::string`/`std::vector` (B-spline control points, records, histories).
  They live on a **small, stable set of entities** (design entity, study
  entity, catalog entity, probe entities) and are read/written by value on
  change, never iterated in hot per-frame paths. Policy: keep them off any
  view used in the frame loop; UI projections read them on dirty.

## 2. Component Catalog

Namespace `exd::sim` (components, all POD where possible; `exd::ecs::Component`
concept applies).

### 2.1 Design

```cpp
struct DesignSpec {                     // parsed, validated engineering intent
    double target_power_w;              // e.g. from "3 MW"
    double rotor_diameter_m;            // optional; derived if absent
    double hub_height_m;                // optional; derived (diameter + clearance)
    double design_tsr;                  // optional; derived from airfoil family
    double cut_in_ms, cut_out_ms, rated_ms;
    double max_thrust_n;                // optional constraint
    double noise_dba_max;               // optional constraint
    int    blade_count;                 // default 3
    std::string airfoil_family;         // "naca5", "custom-catalog"
    double weibull_k, weibull_a;        // site wind distribution (AEP)
    double turbulence_reference;        // reference TI [0..1]
    LoadClass load_class;               // IA/IB/IIA ... (constraint envelope)
    std::string notes;                  // original prompt text
};

struct ParametricModel {                // geometry parameters (serializable JSON)
    std::vector<double> twist_control;  // B-spline control points, deg
    std::vector<double> chord_control;  // B-spline control points, m
    double root_radius_frac, tip_radius_m;
    double cone_deg, tilt_deg;
    double hub_radius_m, hub_length_m;
    double nacelle_length_m, tower_height_m, tower_base_m, tower_top_m;
    int    blade_count;
    int    airfoil_catalog_entry;       // -1 = parametric NACA
    std::array<double,3> naca_params;   // camber, position, thickness
};

struct AirfoilRef { uint32_t polar_id; };   // look-up into airfoil DB
struct ImportedCAD { std::string path; bool normalize; }; // assimp load request
struct GeometryEntity { uint32_t mesh_handle; Bounds3 bounds; bool voxel_dirty; };
```

### 2.2 Study

```cpp
enum class AccuracyTier : uint8_t { T0_Surrogate = 0, T0_BEMT, T1_LBM, T2_LBM_Refined };

struct DesignVariable {
    std::string name;                 // stable id, e.g. "twist.bp3"
    double lo, hi;                    // bounds (CMA-ES space)
    bool discrete;                    // else continuous on [0,1] mapped
    int discrete_count;
    std::string unit;
};

struct Objective { std::string name; double weight; bool maximize; };
struct Constraint { std::string name; double max; double soft_margin; };

struct StudyDefinition {
    std::vector<DesignVariable> variables;
    std::vector<Objective> objectives;
    std::vector<Constraint> constraints;
    AccuracyTier initial_tier, final_tier;
    int population_size;
    uint64_t max_evals_t0, max_evals_t1, max_evals_t2;
    double promote_quantile;          // elite fraction moving up a tier
    std::string algorithm;            // "cmaes" | "nsga2" | "pattern"
    std::string wind_distribution;    // "weibull"
};
```

### 2.3 Mesh / solvers / results

```cpp
struct VoxelGrid {
    int nx, ny, nz;                   // lattice dims
    float dx;                         // lattice spacing (SI)
    uint32_t flags_buffer;            // solid/fluid/BC flags SSBO
    uint32_t u_buffer, rho_buffer;    // D3Q19 + rho SSBOs (ping-pong pair)
    uint32_t force_buffer;            // momentum-exchange accumulators
    Bounds3 world_bounds;
    uint64_t revision;                // bumped on regrid/live edits
};

struct SolverRuntime {                // per-solver core instance
    uint32_t core_id;                 // index into app-owned solver table
    AccuracyTier tier;
    bool enabled;                     // mode/system gating
};

struct SolverStatus   { uint8_t phase; };   // Idle|Running|Converged|Diverged|Cancelled
struct SolverStats    { uint64_t step; double residual; double time_s;
                        double force_x, force_y; double power_w; double cp, ct; };

struct Field3D_Ref    { uint32_t buffer_handle; int nx, ny, nz; float dx; };
struct FieldSurface_Ref { uint32_t mesh_handle; /* subset id */ int subset; };

struct InflowConfig {
    double wind_speed_ms;             // live-adjustable
    double turbulence_intensity;      // [0..0.3] live-adjustable
    double shear_exponent;            // power-law profile
    double yaw_deg;
    uint64_t edited_seq;              // bumped on any change
};

struct TurbineControl { double tsr; double pitch_offset_deg; bool brake; double rpm; };

struct OptimizerState {
    uint64_t generation, total_evals;
    uint8_t algo;                     // cmaes/nsga2/pattern
    uint32_t population_entity;       // container entity holding members
    std::vector<double> best_params;  // best-so-far (normalized)
    double best_fitness;
    bool running, paused;
    uint64_t rng_seed;
};

struct FitnessRecord {                // canonical study-level eval record (§5.5)
    uint64_t eval_id, candidate_id;
    double fitness;                   // weighted scalar (primary)
    std::vector<double> objectives;   // order/semantics per StudyDefinition schema
    std::vector<double> constraints;  // order/semantics per StudyDefinition schema
    AccuracyTier tier; double cost_us;
    bool converged, degraded;         // mirror SolveResult for penalty logic
    uint32_t field_dump_handle;       // 0 = none
};

struct Probe { Vec3f local_offset; std::vector<float> history; uint32_t max_history; };
struct SlicePlane { Vec3f origin; Vec3f normal; Vec2f extent; uint32_t texture_handle; bool dirty; };
struct IsoSurfaceSpec { float value; uint32_t mesh_handle; bool dirty; };
struct StreamlineSpec { uint32_t particle_buffer; int count; float seed_box[6]; };
struct DerivedFieldSpec { uint8_t kind; /* Vorticity|QCrit|WakeDeficit|Cp|Ct */ };

struct ExportRequest {
    std::string geometry_path;        // .stl/.step
    std::string fields_path;          // .vtk/.csv
    std::string report_path;          // .json
    bool in_progress; float progress;
};
```

### 2.4 Render bridge components (written by RenderPreparation systems)

Reuse `exd::render` components directly: `Transform`, `RenderableComponent`,
`Material`, `ParticleCloudComponent`, `VolumeFieldComponent`,
`SimulationDomain`, `RenderTechnique_*` tags. The simulator system layer is the
only writer of these for domain entities; spatial-ui writes its own.

## 3. System Catalog

Namespace `exd::sim::sys`. Each system registers in the app `SystemGraph`
(`graph.add<T>(...).always()/.in_mode(...)`). The **stage** column is a
documented ordering convention — heavy compute in `Simulation`-stage, scene
materialization in `RenderPreparation`-stage; within a stage, registration
order plus explicit `order_before(name, name)` calls define execution order
(see `plan.md` §4.2). If `exd::core`'s phase enum is exposed through the app
graph, the stage maps onto `SystemPhase::{Input, Simulation,
RenderPreparation}` directly.

| System | Stage | Responsibility |
|---|---|---|
| `PromptIntakeSystem` | Input | Routes prompt text to `SpecParser` (rules/LLM); writes `DesignSpec`, diagnostics |
| `AutoConfigSystem` | Input | `DesignSpec` → `StudyDefinition` + `ParametricModel` defaults + `InflowConfig`/`TurbineControl` defaults |
| `ParametricGeometrySystem` | Simulation | `ParametricModel` → `GeometryEntity` mesh (regenerate on variable change) |
| `CADImportSystem` | Simulation | Backgound assimp load → `GeometryEntity` (+ render mesh) |
| `VoxelizationSystem` | Simulation | `GeometryEntity` → `VoxelGrid` flags (+ regrid on revision) |
| `BEMTSolverSystem` | Simulation | `StudyDefinition`+params → `BEMTCore` batch jobs; publishes `SolverStats`, `FitnessRecord`s |
| `LBMSolverSystem` | Simulation | Steps `LBMCore`; rewrites inlet BCs on `InflowConfig.edited_seq`; forces/power → `SolverStats` |
| `StructuralSolverSystem` | Simulation | Loads → `BeamCore` → deflection/stress → constraints |
| `TurbulenceInflowSystem` | Simulation | `InflowConfig` → synthetic turbulence field on inlet (GPU or CPU, reusable) |
| `OptimizerSystem` | Simulation | Advances optimizer: sample → eval batch (via solver systems) → update state; bounded per frame |
| `SurrogateSystem` | Simulation | GP fit on `FitnessRecord` catalog; prediction + uncertainty for promotion decisions |
| `FidelitySchedulerSystem` | Simulation | Assigns tier per candidate; manages promotion queue |
| `StreamlineSystem` | RP | Seed/advect particles on GPU; writes particle buffers |
| `SliceExtractSystem` | RP | Samples `Field3D_Ref` → texture → `SlicePlane.texture_handle` |
| `IsoSurfaceSystem` | RP | Marching cubes (compute or CPU) → mesh |
| `ProbeSystem` | RP/Sim | Samples fields at probe points; appends history |
| `DerivedFieldSystem` | RP | Vorticity/QCriterion/wake deficit via compute |
| `ForceReportSystem` | RP | `SolverStats` → plot-ready series (spatial-ui data graph) |
| `LiveControllerSystem` | Simulation | Applies mode-gated live semantics: T0 refresh instant, LBM re-converge, rotor RPM from TSR |
| `ExportSystem` | Simulation | Runs `ExportRequest`s on background workers; progress updates |
| `WorkbenchProjectionSystem` | RP | Projects domain state into spatial-ui documents (panels/plots) |

System registration pattern: `graph.add<LBMSolverSystem>(gfx, compute, core_table).always()`
or mode-gated via the app's existing mode machinery where a system is
mode-specific (e.g. optimizer only in Optimize mode).

## 4. Concurrency Model

### 4.1 Threads

```
main thread    ECS registry · UI (spatial-ui) · render (GL context owner)
                    │
                    ├─ solver worker pool (BEMT/beam): owned by cores,
                    │    parallel_for over candidates; double-buffered
                    │    result slots with std::atomic publish flags
                    ├─ GPU compute: dispatched from main thread (GL),
                    │    async on driver; results consumed via
                    │    glMemoryBarrier before readback/visualization
                    └─ task queue (std::async/thread): LLM calls, assimp
                         import, exports, surrogate training
```

Rules:

- Registry and render state are touched **only** on the main thread.
- A worker writes into a **preallocated result slot** owned by the core, then
  sets an atomic `ready` flag. The wrapping system consumes committed slots on
  the main thread and clears flags. No locks in the ECS path.
- GPU: one GL context on the main thread. Compute dispatches are batched
  (one `mlx::dispatch` per solver step, one per field op). Readback for stats
  uses PBO + `glMapBufferRange(GL_MAP_READ_BIT)` amortized (e.g. every 500
  steps, mirroring the old FluidX3D wrapper's stability monitor cadence).
- Optimization: `OptimizerSystem` computes `batch = clamp(…, frame_budget)`
  evaluations per frame; candidates are evaluated by enqueueing jobs into the
  BEMT core's pool. GPU-tier evals run on their own `VoxelGrid` ping-pong
  state and are serialized per candidate (a T1 eval is 5–100 ms of compute
  work; 1–2 per frame is plenty).

### 4.2 Determinism

- One `exd::core::Random` (PCG32) per optimizer run, seeded from
  `OptimizerState.rng_seed`.
- Worker pools use per-thread RNGs seeded from the master seed + thread index.
- Fixed iteration orders for systems that touch the catalog.

## 5. Compute Layer (`compute/`)

Typed wrappers over extropian-render's (to-be-added) compute substrate:

```cpp
class GpuBuffer {                 // SSBO with persistent mapping
    uint32_t handle; size_t bytes;
    uint8_t* map(std::span<size_t>);  // glBufferStorage + glMapBufferRange
    void unmap(); void bind(GLuint index);
};
class ComputePipeline {           // compute program + uniform set
    void dispatch(uvec3 groups);
    void barrier(GLenum bits = GL_SHADER_STORAGE_BARRIER_BIT);
};
class PingPong { GpuBuffer a, b; void swap(); };
class VolumeTexture {             // 3D ITextureSource: upload/update slices
    uint32_t handle; int nx, ny, nz; uint32_t update_region(Bounds3i);
};
```

- All GL calls go through render-owned managers (GraphicsContext); `compute/`
  holds no raw GL context state.
- Kernel sources (GLSL) live in `src/physics/lbm/kernels/` and are compiled
  through the extended `ShaderManager`; hash-versioned for cache invalidation.
- Texture viz path: solver SSBO → (optional) `VolumeTexture::update_region` →
  `RenderSystem` volume pass samples it.

## 5.5 Evaluation Data Flow (canonical contract)

One contract — referenced from `solvers.md` §1, `optimization.md` §6, and
`postprocessing.md` §3:

```text
OptimizerSystem                     Solver systems
  design vector x  ───────────────▶  job = SolveJob{id, tier, x→geometry…}
                                     core.solve(job)  →  SolveResult{id, raw physics}
  record = FitnessRecord             mapping per StudyDefinition schema:
    objectives[]  ◀──────────────────   named slots ← SolveResult fields
    constraints[] ◀──────────────────   named slots ← beam core results + arms
    fitness       ◀──────────────────   weighted aggregate (optimizer computes)
    converged/degraded ◀───────────    SolveResult.converged/degraded
```

- `SolveResult` is the **raw core output** (power, thrust, torque, deflection,
  stress, cp, ct, steps, converged, degraded, optional field dump handle). Solver
  systems publish them into the solver cores' ready slots.
- `FitnessRecord` is the **canonical study-level record**. The objective and
  constraint arrays are ordered *by `StudyDefinition`* (the same order the
  UI constraint inspector and the GP surrogate use); the schema lives with
  the study, e.g. `objectives[0] = "aep_mwh"`, `constraints[0] = "max_stress_pa"`,
  `constraints[1] = "tip_clearance_m"`. The `OptimizerSystem` does the
  `SolveResult → FitnessRecord` mapping when it consumes ready slots.
- `SolverStats` is **transient telemetry only** (step, residual, forces) for
  live UI ring-buffers; never persisted in the fitness catalog.
- A non-converged or degraded eval does not crash the run: it enters the
  catalog with `converged=false` and a penalized `fitness`, and the surrogate
  is allowed to learn the failure boundary.

## 6. Frame Pipeline (60 Hz target)

```text
Frame start
 ├─ Input: poll events → spatial-ui interaction → sim input systems
 ├─ Simulation (budget ≤ 8 ms main-thread):
 │    ├─ BEMT batch (pool async; consume ready slots)
 │    ├─ LBM 1–4 steps (GPU; non-blocking dispatch, amortized readbacks)
 │    ├─ optimizer batch (µs evals)
 │    ├─ live controller (BC rewrites trigger LBM re-converge)
 │    └─ derived fields (compute)
 ├─ RenderPreparation (≤ 2 ms): mesh/particle/texture updates, workbench doc
 ├─ Render: exd::render passes (world + UI)
 └─ Frame end: swap, stats
```

Budgets are instrumented per system (`Stopwatch` from core); overshoot logs
and, in CI, fails the performance test.

## 7. Error Handling and Numerical Safety

- Solver steps wrap in a check: any NaN/Inf in fields or stats →
  `SolverStatus::Diverged`; the LBM core keeps the last good snapshot for
  rollback; optimizer marks the candidate `fail` (fitness = worst) rather than
  crashing the run.
- BEMT residual loop caps iterations; non-converged stations contribute
  penalized terms.
- Live edits validate ranges (wind ≥ 0, TI ∈ [0, 0.35]) at the system
  boundary; invalid edits are dropped with a UI notification.

## 8. Serialization and Study Files

- `StudyDefinition`, `ParametricModel`, `InflowConfig`, `OptimizerState`
  (mid-run checkpoint) serialize to JSON (nlohmann, already transitive via
  core). Study files land in `studies/` (gitignored) with schema version
  field.
- `FitnessRecord` catalog persists per run as CSV for post-run analysis.
- Export package layout is specified in `postprocessing.md` §4.

## 9. Testing (architecture-level)

- Pure-core tests do not touch ECS or GPU: BEMT/beam/turbulence cores have
  their own test targets with analytic references.
- GPU tests run behind a `SIM_ENABLE_GPU_TESTS` option and use the compute
  substrate; the CI CPU fallback (LBM on CPU reference kernels, slower but
  correct) keeps integration tests portable.
- Headless: `IRenderer::create(Backend::Null)` fix (ecosystem.md §3) lets the
  full system graph run without a window for prompt→optimize→export CI.