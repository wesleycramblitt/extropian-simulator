# Solvers — Physics Stack and Accuracy Tiers

> Custom solver cores in `src/physics/`. No external solver dependencies
> (FluidX3D is removed). Everything is built for *optimization throughput*
> first: a solver's value = accuracy per unit wall time, and each tier states
> its accuracy envelope explicitly (see `plan.md` §9).

## 1. Common Solver Contract

```cpp
struct SolveJob {
    uint64_t id;
    AccuracyTier tier;
    const ParametricModel* geometry;   // or design-vector → geometry mapping
    InflowConfig inflow;
    TurbineControl control;
    std::span<const float> extra;      // fidelity knobs (grid level, steps…)
};

// Raw physical output of a solver core. No fitness/study concepts here.
struct SolveResult {
    uint64_t id;
    double power_w, thrust_n, torque_nm;
    double max_deflection_m, max_stress_pa;   // empty unless structural ran
    double cp, ct;                     // coefficients at reference conditions
    uint64_t steps; double wall_time_us;
    bool converged, degraded;          // degraded = ran but outside envelope
    uint32_t field_dump_handle;        // 0 = none (optional snapshot)
};
```

Cores are stateless across jobs (thread-safe); GPU cores own their device
state per `VoxelGrid` revision and reuse allocations across jobs. The mapping
`SolveResult → FitnessRecord` (study-level objectives/constraints) happens in
the wrapping ECS systems per the study schema — see `architecture.md` §5.5;
`SolverStats` is transient telemetry for live UI only.

## 2. Tier T0 — BEMT (Blade Element Momentum Theory)

**Purpose**: the workhorse of early optimization — evaluate 5 000–50 000
candidates per run. CPU, thread-pool parallel across candidates.

**Model**:

- Rotor annuli from root to tip (default 20–40 elements; variable count is a
  fidelity knob).
- Momentum theory (axial + tangential) with the standard corrections:
  - Prandtl tip/hub loss factors;
  - Glauert high-thrust correction (empirical `a > a_crit` branch);
  - Skewed-wake correction for yaw (Glauert/Pitt–Peters).
- Blade element forces from 2-D polars `Cl(α), Cd(α)` (airfoil DB,
  Reynolds-scaled by chord·relative-velocity).
- Steady-state solve per annulus (fixed-point iteration with under-relaxation,
  capped iterations; non-converged stations penalize rather than fail).
- Operates over the required wind-speed grid (cut-in → cut-out, default 21
  points) to produce the power curve; AEP = ∫ P(U)·f_weibull(U) dU.
- Structural block: T0 optionally feeds loads to the beam core (§5) for
  deflection/stress constraints at 3–5 evaluation wind speeds only.

**Cost**: 10–100 µs/candidate (power curve + loads). The thread pool saturates
all cores; 10 000 evals ≈ 0.1–1 s of *compute* wall time (end-to-end budgets
including optimizer bookkeeping: `optimization.md` §7).

**Accuracy envelope**: power ±15–30% (site-dependent; best on steady, attached
operating points), thrust/loads ±20–40%, fails qualitatively at high TI and
deep stall. This is a *ranker*, not a reporter — the optimizer uses it to
order candidates; absolute values come from higher tiers.

**Validation**: constrained re-optimization against NREL 5 MW (golden test)
and published power curves (e.g., DTU 10 MW class curves in `tests/golden/`).

## 3. Tier T1 — ALM-LBM GPU (Actuator-Line LBM)

**Purpose**: mid/late optimization tiers and the **live run mode**. Wake- and
farm-scale physics at interactive cost.

**Why actuator line at T1**: a uniform grid that simultaneously resolves the
blade chord (≥ 4–8 voxels) *and* a full-size rotor domain (≥ 8 D streamwise,
≥ 3 D lateral) requires 10³–10⁴× more cells than interactive budgets allow at
any turbine scale. The standard engineering answer — used by LES wind codes
(SOWFA, AMBER, EllipSys) — is to represent the rotor as **body forces**: BEMT
section loads are projected onto the lattice as momentum source terms along
the blade lines. The wake and its interaction with turbulence, shear, yaw and
the tower are then fully resolved by the LBM without resolving blade
geometry. Blade-resolved simulation is moved to T2 where refinement is local.

**Model**:

- D3Q19 velocity set, FP32 with lattice-unit normalization (standard LBM
  practice).
- Collision: BGK base with TRT (two-relaxation-time) default; Smagorinsky
  LES-lite (`ν_t = (C_s Δ)²|S|`).
- **Actuator line**: each blade is a line of body-force nodes (20–40, aligned
  with the BEMT stations). At every step the BEMT section-load model (same
  core as T0, evaluated at the *resolved* local inflow sampled from the
  lattice at the actuator points) produces lift/drag forces; they are smeared
  onto the lattice with a 3-D Gaussian kernel (ε ≈ 2–3 Δ) and applied as an
  external force in the collision step. Actuator nodes advance with
  `TurbineControl` ω — no regrid on speed changes.
- Tower/hub/nacelle: v1 models the tower as a drag body-force line; nacelle
  as a smoothed solid box (±3Δ). Voxelized tower geometry is a T2 option.
- Boundary conditions:
  - Inlet: prescribed velocity profile (power-law shear) + synthetic
    turbulence (§6) perturbation layer;
  - Outlet: convective/zero-gradient + pressure anchor;
  - Top/bottom/sides: free-slip; domain ≥ 8 D streamwise, ≥ 3 D lateral
    (D = rotor diameter), hub-centered.
- Reported forces: blade loads come from the BEMT section model at the
  resolved inflow (standard ALM output); the LBM contributes wake, induction
  and turbulence interaction; LBM-resolved pressure on the (coarse) tower
  voxels contributes tower loads at T1 only nominally (validated in the
  wind-tunnel case).

**Cost** — explicit throughput assumption: **1 GLUPS** (10⁹ lattice updates/s)
on a mid-range desktop GPU for fused D3Q19+TRT+Smagorinsky FP32 SoA kernels
(achieveable with the buffer layouts in `ecosystem.md` §3; benchmark target
`benchmarks/mlups`; conservatively 300 MLUPS minimum on integrated GPUs):

| Grid (uniform, wake-resolving) | Cells | Per-step cost | Steps to settle (warm-started) | Wall time to settle |
|---|---|---|---|---|
| 96³ | ≈ 0.9 M | ≈ 1 ms | 2k–8k | ≈ 2–10 s |
| 160³ (large rotor / higher fidelity) | ≈ 4 M | ≈ 4–5 ms | 4k–12k | ≈ 15–60 s |

**Live mode** (details in `postprocessing.md`): 1–4 steps/frame ⇒ the wake
evolves at 1–4 rotor diameters of convection per second on screen: visibly
responsive within ~1–3 s after a control change, fully settled in tens of
seconds for utility-scale domains. Integrated quantities (power, thrust,
Cp/Ct) come from T0/BEMT and update **instantly**; the LBM resolves the
*distribution* (wake, induction field, loads under turbulence). This is
transient, near-real-time physics, not converged steady state — the UI's
"settled" badge shows the difference (`postprocessing.md` §1).

**Accuracy envelope**: Cp ±15–25%; wake centerline deficit and turbulence
profiles ±20–30% vs wind-tunnel/verified LES at matched Reynolds and TI;
rotor loads ±20–40% (they inherit BEMT section-model error). Degraded in deep
stall, very high TI (>25%), and low Reynolds where transition matters. Not a
blade-pressure tool — that is T2/T3.

**Validation**: Taylor–Green decay; Poiseuille channel (exact); lid-driven
cavity (Ghia et al.); ALM-LBM rotor case vs published periodicity-averaged
wake data (MEXICO / DTU tunnel cases, NREL 5 MW golden test at operating
point).

## 4. Tier T2 — Blade-Resolved LBM (local refinement)

**Purpose**: final in-engine verification before export; the accuracy bridge
to external CFD.

**Model**: same LBM core with

- **Local refinement**: 2–3 nested refinement levels (factor-2 per level)
  around the blade swept volumes and the near wake; base grid 96³–128³ ⇒
  effective blade resolution 4–8 voxels/chord. Standard LBM grid coupling
  (restriction/prolongation at level interfaces).
- **Rotating geometry**: blade/hub surfaces rotated per step in the refined
  region (bed-of-nails rotation with one-cell smoothing), or a rotating-frame
  rotor subdomain — whichever validates better on the golden test.
- Curved no-slip walls (interpolated bounce-back) at the blades; tower/hub
  voxelized as before.
- Optional 2nd-order outflow.

**Cost**: refined grids ≈ effective 192³–320³ → settle (warm-started from the
T1 field, `optimization.md` §5.3) in **10–60 s** per evaluation; 1–5 evals
per study. The optimizer treats T2 as rare and final (≤ 10 per run).

**Accuracy envelope**: Cp ±5–10%; blade loads ±10–20% in attached flow; wake
±15% in the refined region. Remaining error sources: Smagorinsky coefficient,
grid COV, turbulence-synthesis statistics — all disclosed in the export
report's accuracy envelope (`postprocessing.md` §4).

**Validation**: NREL 5 MW operating points vs published results; MEXICO rotor
pressures/forces; one mesh-convergence check per study.

## 5. Structural Core — Timoshenko Beam (T0/T1/T2 loads)

**Purpose**: constraint evaluation (max stress, deflection, tip clearance) and
refinement loop for aero-elastic consistency.

**Model**:

- Per blade: variable-section Timoshenko beam, 2-D (flap/lag) with torsion
  (pitch axis) — 20–40 stations matching BEMT/ALM element layout.
- Material properties from `exd::physics::MaterialDatabase` (existing) +
  composites table (fiberglass/carbon layup stiffness and density inputs from
  `assets/materials/`).
- Loads: steady + deterministic gust envelope (IEC-class based on
  `DesignSpec.load_class`), plus gravity.
- Solve: direct stiffness assembly, small system (≤ 120 DOF per blade) —
  microseconds; runs on the worker pool alongside BEMT.
- Failure: max-stress (Tsai–Wu-lite via material allowables) and deflection
  limit (tip clearance to tower at maximum deflection).

**Accuracy envelope**: deflection ±15%, natural-frequency estimate ±20%
(needed for aero-elastic excitation checks). Not a fatigue tool — final
fatigue lives are exported for external FEA (T3).

## 6. Turbulence Inflow Core

**Purpose**: physically plausible, cheap, live-regenerable inflow turbulence.

**Model**:

- Synthetic spectral turbulence: Mann/Veers-style — a divergence-free
  velocity field synthesized from the Kaimal spectrum (or Mann tensor) with
  Taylor's frozen-turbulence advection. Parameters: `turbulence_intensity`,
  length scale (derived from hub height / rotor diameter), shear profile.
- GPU-friendly: one compute pass generates the perturbation box by summing
  Fourier modes (configurable mode count, default 64–256) over the inlet
  plane; seeded per run for determinism.
- Live adjust: TI/length-scale changes regenerate the mode coefficients
  (µs–ms), no solver restart.
- T0 path: TI and shear fold into BEMT as analytical corrections (empirical
  TI attenuation at rotor plane).

**Accuracy envelope**: energy content within ±25% of the target spectrum
between Λ/10 and 10Λ (adequate for wake dynamics at T1); not intended for
extreme-load statistics — that is T3 verification domain.

## 7. Meshing / Voxelization

- T1 needs no body mesh: the actuator line plus optional coarse tower/nacelle
  voxels rasterize directly from `GeometryEntity` bounds. T2 voxelizes the
  blade-swept volume with local refinement.
- Algorithm (T2): coarse-to-fine octree sample pass (watertight test per cell
  from ray parity), then flood-fill interface classification (fluid/solid/
  inlet/outlet/symmetry). Flags live in the `flags_buffer` SSBO; cached per
  `VoxelGrid.revision`.
- Cost: T2 voxelization ≈ tens of ms (multithreaded CPU; GPU fallback later).
- The parametric generator emits closed, manifold surfaces by construction;
  imported CAD is cleaned (merge vertices, weld, closed-boundary check)
  before voxelization.

## 8. Accuracy Tier Policy (recap)

| Tier | Code | Where in product |
|---|---|---|
| T0 | BEMT + beam + TI corrections | surrogate training, rankings, constraint pre-screen |
| T0.5 | BEMT corrected by GP surrogate against T1 samples | main optimization loop after warm-up |
| T1 | ALM-LBM 96³–160³ + beam | convergence of top candidates, live run |
| T2 | Blade-resolved LBM (refined) | finalists, export figures |
| T3 | External high-fidelity | verification & fine tuning (out of engine) |

The multi-fidelity scheduler in `optimization.md` §4 decides promotion; the
rules engine in `plan.md` sets initial tiers from `DesignSpec` (e.g., noise
constraint active → T1 mandatory for representative loads).

## 9. Future Physics Domains

The core contract (§1) is domain-agnostic: replace "rotor" with "device",
extend `SolveResult` with domain-specific integrals. Candidate next domains
(see `plan.md` Phase 8): duct/heat-exchanger flow (same LBM core, different
BCs — and the actuator-line trick generalizes to any immersed source term),
acoustics (LBM + Ffowcs-Williams–Hawkings post-process), multibody mechanisms
(beam + rigid-body kinematics), electromagnetics-lite (2-D FEM on the same
lattice). Each lands as a new core behind the same `SolveJob/SolveResult`
contract with its own tier table.
