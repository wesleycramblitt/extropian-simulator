# Extropian Simulator

The **interactive simulation workspace**: a single app where a parametric
CAD design is edited directly in a 3D view, real-time simulations run
against it, results stream back into spatial (in-world) dashboards, and
optimization loops drive the design — all without leaving the scene.

**For now this is a stub.** `./build/extropian-simulator-workspace`
(one `main.cpp`, see `workspace/`) boots the workspace UI/UX — window,
3D viewport, ImGui host — with the ECS seeded with placeholder domain
state for the four axes (CAD design, live simulation, visualization,
optimization). The real workspace layers depend on
`extropian-spatial-ui` (spatial document/dashboard pipeline) and the
interactive editing path in `extropian-geometry`, neither of which is
ready yet; they will replace the stubs as they land.

## What's in the repo

| Piece | Where | Status |
|---|---|---|
| Workspace app (stubbed) | `workspace/main.cpp` | boots UI/UX + seeded ECS |
| `exd::sim` library | `include/exd/sim/`, `src/` | ECS simulation systems (turbine, steam engine, solver runs, shape recipes, dashboard feed) |
| Headless tests | `src/*_test.cpp` | deterministic recipes: coupled FDM3, 0D engine, dashboard pipeline |

The old registration demos were removed — the workspace app is now the
single entry point. Experiments still live in
[`extropian-playground`](https://github.com/wesleycramblitt/extropian-playground).

## Building / running

```bash
./build.sh          # library + tests + workspace app
./run.sh            # launch the workspace (builds on demand)
# headless tests:
./build/optimization_test   ./build/shape_workshop_test
./build/solver_run_test     ./build/engine_run_test   ./build/dashboard_feed_test
```

Local sibling checkouts under `../` are picked up automatically by
`build.sh`; `extropian-assets` (media) is always fetched from GitHub.

## Ecosystem

`extropian-core` (ECS/math) · `extropian-render` (pipeline/ImGui) ·
`extropian-app` (window) · `extropian-physics` (solvers) ·
`extropian-geometry` (CAD recipes) · `extropian-optimization` ·
`extropian-viz` (fields/colormaps) · `extropian-spatial-ui` (dashboards,
early) · `extropian-playground` (experiments). Conventions and component
ownership: [`AGENTS.md`](AGENTS.md).

## License

Business Source License 1.1 — see [LICENSE](LICENSE).
Converts to Apache 2.0 on 2029-05-26.
