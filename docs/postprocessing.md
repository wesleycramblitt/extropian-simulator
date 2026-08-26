# Post-Processing and Live Run

> This repository owns all post-processing logic: the live interactive run,
> visualization pipeline, probes/plots, and the verification export package.
> Render happens through `exd::render::RenderSystem`; UI chrome comes from
> spatial-ui (`postprocessing.md` is the contract for both).

## 1. Live Run — the Post-Optimization Product

After optimization accepts a winner, the app enters **Live Run** mode: the
winning `ParametricModel` is materialized, its `VoxelGrid` is built at T1
resolution, and the LBM settles to steady state in the background while the
user inspects. All environment and control parameters are live:

| Control | Range | Update semantics |
|---|---|---|
| Wind speed | 0–60 m/s (step 0.1) | T0 quantities (power, thrust, Cp/Ct) recompute instantly (BEMT ~50 µs); LBM inlet BCs + actuator-line forces rewritten → wake re-equilibrates visibly in ~1–3 s, settles in 2–10 s (T1 uniform) |
| Turbulence intensity | 0–0.35 | Turbulence core regenerates mode coefficients (ms); inlet perturbation layer swapped; wake re-develops ~1–3 s visibly |
| Shear exponent | 0–0.5 | Recompute inlet profile; instant on T0, quick on LBM |
| Yaw | −30°…+30° | BC rotation; skewed wake develops in seconds |
| Rotor TSR / RPM | 0–1.5 × design | Actuator nodes advance at new ω (no regrid); power/forces track live |
| Pitch offset | −5°…+10° | BEMT instant; ALM section pitch applied at next actuator force eval |
| Brake | on/off | Hard stop → wake collapse demo |

Every control change bumps `InflowConfig.edited_seq` / `TurbineControl.edited_seq`;
systems react only to sequence deltas, so identical values cost nothing.

**Transient vs settled semantics**: the live run is a *transient,
near-real-time* T1 simulation (1–4 steps/frame) — the wake is always visibly
evolving, not converged. Integrated quantities are authoritative from T0
(instant); the LBM provides the resolved *distribution* (wake, induction,
loads under turbulence). The "settled" badge (residual < threshold, sustained
N steps) tells the user when the current transient is close to steady — it is
part of the accuracy contract, not cosmetics.

## 2. Visualization Pipeline

All visualization is computed by post-process systems in RenderPreparation,
writing the render components extropian-render already consumes.

### 2.1 Field plumbing

LBM state lives in SSBOs (`VoxelGrid.u_buffer/rho_buffer`). Two routes:

- **Texture route** (slices, iso, volume): `VolumeTexture::update_region`
  copies field → 3D texture (RGBA16F or R32F per field); `RenderSystem`
  volume pass (needs the real ray-march shader; see `ecosystem.md` §3)
  samples it. Decimation: update a coarse probe texture every frame, a full
  texture at 1–4 Hz.
- **Buffer route** (streamlines, glyphs): compute shaders read the SSBOs
  directly.

### 2.2 Representations

| Representation | System | Method | Render path |
|---|---|---|---|
| Streamlines / pathlines | `StreamlineSystem` | GPU particle advection (RK2) of seeded particles through `u` SSBO; re-seed on inflow change | `ParticleCloudComponent` + particle pass (existing) |
| Slices (U, P, vorticity) | `SliceExtractSystem` | Compute shader samples field → 2-D texture per `SlicePlane` | textured quad mesh (Lambertian w/ texture or UI overlay) |
| Iso-surfaces | `IsoSurfaceSystem` | Compute marching cubes (small grids) or CPU marching cubes at LOD; rebuilt on revision | `MeshManager::create` + `RenderableComponent` |
| Surface contours | sim → `MeshAssetSystem` path | vertex colors from field | Lambertian with vertex colors |
| Volume rendering | (render) | ray-march shader over 3D texture; transfer function from `DerivedFieldSpec` | volume pass (render) |
| Probes | `ProbeSystem` | CPU sample from latest readback or compute shader reduction | spatial-ui scatter/sparkline meshes |
| Forces / power / Cp-Ct | `ForceReportSystem` | from `SolverStats` | spatial-ui plots (line/bar) + HUD |
| Rotor animation | `LiveControllerSystem` | rotates blade entities by ω·t from control | standard render |

Level of detail policy: T1 steady mode renders everything; during re-converge
after a control change, streamline counts halve and the volume texture drops
to probe-LOD until settled — keeps the frame budget flat while the wake
evolves.

### 2.3 Derived fields

`DerivedFieldSystem` (GPU): vorticity ω = ∇×u, Q-criterion, wake-deficit
profiles at station planes, Cp/Ct extraction per blade from force buffers.
Derived outputs are `Field3D_Ref`/`FieldSurface_Ref` components consumed by
the representations above.

## 3. Probes, Plots, and Data Flow to UI

- Probes: fixed local offsets (or gizmo-moved in the viewport); each probe
  writes a ring-buffer history (`Probe.history`) consumed by the
  `WorkbenchProjectionSystem` into spatial-ui plot documents
  (`generate_scatter_mesh`, `generate_line_mesh`, `generate_sparkline_mesh`).
- Power curve panel: live BEMT over the wind grid; the operating point marker
  reads from `InflowConfig.wind_speed_ms`.
- Convergence panel: residual history (`SolverStats`) as sparkline.
- Pareto/candidate history: from `FitnessRecord` catalog (Optimize mode).

All numbers flow ECS → view models → spatial-ui documents; this repo owns the
view models, spatial-ui owns rendering of the widgets.

## 4. Export / Verification Package

Triggered from Export mode (`ExportRequest`):

| Item | Format | Content |
|---|---|---|
| Geometry | STL (v1), STEP (v2, tessellated) | rotor assembly, tower, hub; parametric metadata sidecar JSON |
| Fields | VTK (legacy/XML) + CSV | final T2 fields on the lattice; probe histories CSV |
| Report | JSON | study definition, variable values, fitness, tier schedule, accuracy envelope claims, solver stats, site inputs |
| Sanity checks | — | power curve vs spec, constraint margins, mesh quality (min angle / aspect ratio via `exd::physics` quality functions), coverage of the load envelope |

The report's accuracy-envelope section is mandatory: consumers in
high-fidelity software must know what the numbers are and are not (e.g.,
"T1 LBM, ±10–15% Cp, no fatigue").

**Golden round-trip test**: exported geometry re-imports via the same
`CADImportSystem` and matches the source mesh within tolerance; VTK/CSV parse
correctly (tests use `exd::physics` io or a reader check).

## 5. Performance Budget (Live Run)

| Activity | Budget |
|---|---|
| LBM T1 steps | 1–4 / frame (~1–8 ms GPU, `solvers.md` §3) |
| Texture route updates | ≤ 1 ms/frame at probe LOD; full field ≤ 1 Hz |
| Streamline advection | ≤ 1 ms/frame (GPU) |
| BEMT instantaneous refresh | ≤ 0.1 ms per control change |
| Probe/plot updates | ≤ 0.5 ms/frame |
| Workbench doc rebuild (dirty-only) | ≤ 0.5 ms/frame |

Instrumented via `Stopwatch` per system; CI performance tests assert the
budgets on the reference machine.