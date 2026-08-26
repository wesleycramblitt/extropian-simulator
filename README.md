# Extropian Simulator

**Real-time multiphysics inverse design and optimization engine.**

Type a prompt like *"Design a 3 MW wind turbine with low noise, 60 m blades,
class IB loads"* — the engine generates a parametric CAD model (or imports
yours), auto-configures fitness functions, meshing, boundary conditions and
solvers, runs aggressive optimization loops with custom GPU-first solvers,
and finishes with a live, interactive post-optimization run where wind speed,
turbulence, yaw and rotor speed can be changed in real time.

## Modes

| Mode | Purpose |
|------|---------|
| **Design** | Prompt/spec intake, geometry generation or CAD import, review auto-configured study |
| **Optimize** | Run optimization loops (CMA-ES / NSGA-II, multi-fidelity BEMT→LBM→refined) with live convergence |
| **Live Run** | Real-time post-optimization simulation: adjustable wind, turbulence, yaw, TSR; streamlines, slices, iso-surfaces, probes, force/power plots |
| **Export** | Verification package: STL/STEP geometry, VTK/CSV fields, study report |

## Architecture

Everything is ECS (`extropian-core`). Solver cores live in this repository
(`src/physics/`: BEMT, GPU lattice-Boltzmann, structural beam, synthetic
turbulence) — no external solver dependencies. Rendering flows through
`extropian-render`; all UI comes from `extropian-spatial-ui`; post-processing
logic is owned here.

See `docs/plan.md` (master plan), `docs/architecture.md`,
`docs/solvers.md`, `docs/optimization.md`, `docs/postprocessing.md`,
`docs/ecosystem.md`.

## Building

```bash
./build.sh          # or: cmake -S . -B build -G Ninja && cmake --build build
./run.sh
```

Local sibling checkouts (extropian-core/render/app/physics/spatial-ui) are
picked up automatically by `build.sh` to avoid GitHub fetches.

## Status

Early-stage: rendering prototype on the shared ECS/render stack; Phase 0–1 of
`docs/plan.md` (foundations + compute substrate). FluidX3D is no longer used.

## License

Business Source License 1.1 — see [LICENSE](LICENSE).
Converts to Apache 2.0 on 2029-05-26.