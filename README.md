# Extropian Simulator

An **interactive simulation workspace**: direct, real-time manipulation of a
parametric design in a 3D view, where geometry edits flow into running
simulations and results stream back into a spatial (in-world) dashboard —
without ever leaving the scene.

The workspace experience is built on two ecosystem pieces that are still
early: `extropian-spatial-ui` (the document/layout pipeline that turns
domain data into spatial UI) and the interactive editing path in
`extropian-geometry`. **Until those mature, this repo ships the foundation
for that experience:**

1. **`exd::sim`** — the ECS-based simulation systems library
   (`include/exd/sim/`, `src/`): turbine / steam-engine / solver-run /
   shape-workshop systems, coupled-CFD and 0D-ephemeral objective models,
   dashboard feed, all governed by the standards in [`AGENTS.md`](AGENTS.md).
2. **Headless integration tests** — deterministic verification of the
   run-mapping recipes against the real solvers (`EXT_SIM_BUILD_TESTS`,
   on by default, OFF when this repo is consumed as a FetchContent
   dependency).

## Registration examples

The windowed demos that used to live here (how to compose `exd::sim`
systems into a full app: window, render pipeline, ImGui panels, solver
systems) **moved to [`extropian-playground`](https://github.com/wesleycramblitt/extropian-playground)**
(`playground/demos/`), which also hosts the broader ecosystem experiment
space. New examples and experiments belong there.

## Ecosystem

```text
extropian-core           ECS, math, config          extropian-physics      solver cores
extropian-render         render pipeline, ImGui     extropian-geometry     parametric geometry
extropian-app            window + frame loop        extropian-optimization CMA-ES / NSGA-II / NM
extropian-viz            fields, colormaps, slices  extropian-spatial-ui   spatial dashboards (early)
extropian-assets         media (fetched, never stored here)
extropian-playground     demos + ecosystem experiments
```

Local sibling checkouts under `../` are picked up automatically by
`build.sh` (no re-fetch from GitHub), except `extropian-assets`, which is
always fetched from GitHub.

## Building

```bash
./build.sh
# headless tests (the shipped verification surface):
./build/optimization_test   ./build/shape_workshop_test
./build/solver_run_test     ./build/engine_run_test   ./build/dashboard_feed_test
```

## Status

- The `exd::sim` library and its recipes are exercised and deterministic
  (coupled FDM3 runs, 0D engine cycles, dashboard feed).
- The interactive workspace itself is **early-stage**: the spatial-ui
  document pipeline and the interactive geometry editing path are not ready
  yet. Until they are, expect the repo to ship library + tests, with
  workspace UI work landing as those dependencies mature.

## License

Business Source License 1.1 — see [LICENSE](LICENSE).
Converts to Apache 2.0 on 2029-05-26.
