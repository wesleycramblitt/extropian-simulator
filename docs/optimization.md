# Optimization Engine

> The product is the *loop*: prompt → thousands of cheap, tiered evaluations →
> converged design. This document details parameterization, fitness,
> algorithms, multi-fidelity scheduling, and surrogates. See `plan.md` §7 for
> the summary and `solvers.md` §1 for the evaluation contract.

## 1. Design Parameterization

### 1.1 Variables

All variables live in `StudyDefinition.variables` (design vector `x ∈ [0,1]^d`
in optimizer space; affine-mapped to engineering units for the solvers).

**Continuous** (typical v1 rotor problem, d ≈ 12–20):

| Variable | Range | Effect |
|---|---|---|
| rotor radius | 0.75–1.25 × spec default | sizing, AEP, loads |
| twist B-spline control points (3–5) | −4°…+8° per station | power capture, stall margin |
| chord B-spline control points (3–5) | 0.5–2 × spec default | solidity, loads, mass |
| airfoil camber / thickness (parametric NACA) | 2–6% / 12–24% | Cl/Cd trade, structural thickness |
| cone / tilt angles | 0–8° | clearance, yaw loads |
| hub height | 0.8–1.4 × spec | AEP via shear, tower cost |
| tower taper (base/top diameters) | geometry bounds | mass, resonance margin |

**Discrete**:

| Variable | Choices |
|---|---|
| blade count | 2 / 3 / 4 |
| airfoil family | parametric-NACA / catalog entries (polar IDs) |
| material class (structural) | fiberglass / carbon / hybrid |

Discrete variables map through a per-variable lookup table into the
optimizer's continuous space (relaxed, nearest-integer penalty at eval time).

### 1.2 Geometry reconstruction

`ParametricGeometrySystem` maps the engineering vector → `ParametricModel` →
`GeometryEntity` mesh + BEMT station table. The mapping is **injective and
smooth** (B-spline control points instead of raw station arrays) so the
optimizer's continuity assumptions hold and every interior point is
physically valid (chord/twist monotonicity constraints folded into the spline
basis where possible).

## 2. Fitness

### 2.1 Objectives (from `DesignSpec` + site)

Default single aggregate (weighted scalar) plus Pareto archive for
multi-objective runs:

1. **AEP** — `∫ P(U) · f_weibull(U; k, A) dU` over cut-in→cut-out; P(U) from
   the tier solver power curve. Primary objective (`maximize`, weight 1.0).
2. **Thrust / extreme loads** — max thrust at rated; constraint (hard) or
   secondary objective when `max_thrust_n` given.
3. **Structural** — max deflection (tip clearance) and max stress vs
   allowables: hard constraints with soft margin (violation within margin
   degrades fitness smoothly; beyond margin → infeasible).
4. **Noise proxy (optional)** — tip-speed-based empirical dBA estimate from
   spec limit (constraint).
5. **Cost proxy (optional)** — blade/tower mass model from geometry + material
   class (secondary objective).

Normalization: each objective normalized by a reference value (from spec or
first evaluated candidate) so weights are dimensionless.

### 2.2 Constraint handling

- Hard: feasibility via penalty barrier `fitness -= w·max(0, v/v_ref − 1)^2`.
- Infeasible candidates are *retained in the catalog* (surrogate needs the
  data) but excluded from parent selection.
- `FitnessRecord.constraints` carries raw values for the UI constraint
  inspector.

### 2.3 Determinism

One seeded RNG per run; all stochasticity (CMA-ES sampling, turbulence seeds,
promotion tie-breaks) derives from it. Re-running a study reproduces the
catalog bit-for-bit (same tier availability).

## 3. Algorithms

### 3.1 CMA-ES (primary)

- Continuous variables only (discrete relaxed); λ = 4 + 3·ln(d) ≥ 20 (batch
  size matches BEMT pool throughput).
- Restart strategy (IPOP): on stagnation, restart with doubled population.
- Termination: budget exhausted, fitness plateau (Δ < ε over N generations),
  or all elite feasible & stable for 3 generations.

### 3.2 NSGA-II (multi-objective)

- Used when ≥ 2 objectives are active or Pareto archive requested.
- Archive persists to `FitnessRecord` catalog; UI shows the Pareto front.

### 3.3 Pattern search (fallback / polish)

- Coordinate/Booth-style pattern search around the CMA-ES solution for local
  polish in the final tier (T2 evals are expensive; dry polish first on
  surrogate).

## 4. Multi-Fidelity Scheduling

```
generation 0 ── population BEMT (T0) ──────────────────────────────── fast
generation k ── top q% promoted → LBM T1 (subset, 1–2 per frame) ──── slow
final ───────── top 3–5 → T2 refined ───────────────────────────────── rare
```

- **Promotion**: elite quantile `promote_quantile` (default 10%), subject to
  surrogate uncertainty: promote when GP-predicted T0→T1 correction is large
  or uncertain (exploration of model error), skip when confident (exploiting
  the correction).
- **Demotion**: T0 rank and T1 rank disagree beyond cardinality threshold →
  re-check with an additional T1 eval before trusting T1.
- **Correction model**: per-candidate T1 sample tunes a GP error model
  `ε(x) = f_T1(x) − f_T0(x)`; T0 evaluations are then fitness-corrected by
  the GP prediction: `f̂(x) = f_T0(x) + ε̂(x)` (T0.5 tier in `solvers.md` §8).
- Budgets are explicit in `StudyDefinition` (`max_evals_t0/t1/t2`) — the
  "aggression" knob the UI exposes as *fast / balanced / thorough*.
- Live mode is outside scheduling: it always runs T1.

## 5. Surrogates and Reduced-Order Models

### 5.1 GP fitness surrogate

- Input: design vector (first ~10 PCA components of the variable space if
  d > 15); output: fitness + error-model value. Matern-5/2 kernel.
- Fit cost: O(n²) per fit with n ≤ 2 000 (subset selection via farthest-point
  sampling beyond n_max); fits run on the task queue, not the frame.
- Used by: promotion decisions, pattern-search polish target, and the
  "what-if" UI (instant fitness estimate while dragging a variable).

### 5.2 Snapshot-PCA ROM of LBM fields

- Periodically (every T1 eval batch, capped) the LBM wake field (u-magnitude
  on a fixed coarse probing grid) is captured as a snapshot.
- Incremental PCA (rank-r, r ≤ 32) maintains the wake mode basis per study.
- Uses: warm-starting the LBM (projective initialization from modes),
  cheap wake-quality fitness (deficit profile from the low-rank field),
  and visualization of "the wake family" for the design space.

### 5.3 Warm start policy

T1/T2 evals initialize from the nearest previously evaluated candidate's field
snapshot (mode-extrapolated) — typical 30–50% step savings at steady
convergence, preserving determinism because the snapshot is part of the study
state.

## 6. Optimizer as ECS

- `OptimizerState` entity holds algorithm state and a `population` container
  entity; each member is a lightweight entity with
  `DesignVector + FitnessRecord` components (catalog entities) — the UI
  tables, plots and selection all read from these.
- `OptimizerSystem` per frame:
  1. consume ready `SolveResult`s → map to `FitnessRecord`s per the canonical
     contract (`architecture.md` §5.5) and update the catalog;
  2. sample next batch (bounded by frame budget and pool capacity);
  3. write progress components (generation, best, front).
- Pause/resume/seed: `OptimizerState.running/paused`; seeding writes
  `best_params` + optional design vector → new run continues from there.
- The optimizer never owns physics: it only produces design vectors and
  consumes `SolveResult`s produced by the solver systems.

## 7. Aggression Targets and Budgets

| Profile | T0 evals | T1 evals | T2 evals | Wall clock (desktop GPU) |
|---|---|---|---|---|
| fast (early exploration) | 5 000 | 30 | 3 | ~1–3 min |
| balanced | 25 000 | 100 | 6 | ~5–15 min |
| thorough | 50 000+ | 300 | 10 | ~20–60 min |

End-to-end budget logic (also protects the UI):

- T0 evaluations are **frame-batched**: `OptimizerSystem` enqueues 500–2 000
  evals per frame to the BEMT worker pool (µs work each; pool saturates all
  cores off-thread, main thread spends ≤ 8 ms/frame on bookkeeping). 50 000
  evals ≈ 30–60 s wall regardless of pool speed.
- T1 evals are serialized 1–2 per frame (each 2–10 s of GPU settle after
  warm-start, `solvers.md` §3); the elapsed time is dominated by settle, so
  the loop stays interruptible and the UI keeps updating.
- T2 evals are rare and final (10–60 s each, `solvers.md` §4).
- Every batch respects the frame budget (≤ 8 ms main thread) — the loop is
  always pause/abort/resume-able from the Optimize mode UI.

These bounds also protect the UI: the loop is always interruptible
(pause/abort), and every batch respects the frame budget.

## 8. Verification of the Loop Itself

- Golden test: constrained re-optimization of NREL 5 MW from a neutral start
  must recover the published rotor within variable space tolerance tier-1
  fitness; guards against optimizer/solver sign errors.
- Artificial-function tests: run the full pipeline on test functions
  (Rosenbrock/Sphere scaled to turbine-like d) to validate optimizer
  plumbing without physics.
- Catalog audit tests: records consistency, no NaN fitness, determinism under
  fixed seed, promotion statistics within expected ranges.