# Demos

One executable per registration example. All demos subclass `DemoApp` and
only **register systems** — every system lives in the `exd::sim` library.

| Demo | What it shows | Systems registered |
|---|---|---|
| `extropian-sim-optimize` (default) | minimal multi-system composition: analytic-objective CMA-ES optimization driving a live turbine design | OptimizationSystem (Analytic), TurbineSystem |
| `extropian-sim-turbine-solver` | real meshing + CFD: full coupled FDM3 solver runs in the background; the flow field is published as a speed slice + streamlines; CMA-ES with one short coupled-CFD run **per candidate**; spatial-ui dashboard | OptimizationSystem (CoupledCfd), SolverRunSystem, TurbineSystem, DashboardFeedSystem + scene_renderer UI pipeline |
| `extropian-sim-steam-engine` | steam engine: parametric machine meshing animated per crank angle, 0D Rankine-lite solver runs, CMA-ES over boiler pressure/cutoff/geometry, indicator dashboard | OptimizationSystem (EngineSim), SteamEngineSystem, DashboardFeedSystem + scene_renderer UI pipeline |

## Run

```sh
./build.sh                 # builds everything
./run.sh                   # default demo (extropian-sim-optimize)
./run.sh extropian-sim-turbine-solver
./run.sh extropian-sim-steam-engine
```

`Z` toggles fly camera / UI mode. The physics demos accept `--auto-run`
(start optimization + a solver run ~0.5 s after boot — a smoke test).

## Copying a demo

`demos/<name>/main.cpp` is the full application; the only hook is
`register_sim_systems(graph)` plus optional `on_update` logic. Add the
executable + asset-copy entry in `demos/CMakeLists.txt`.

## Headless tests

```sh
./build/optimization_test      # CMA-ES objective (analytic reference)
./build/solver_run_test        # coupled-run recipe: valid + deterministic
./build/engine_run_test        # engine-run recipe: valid + deterministic + sensitivity
./build/dashboard_feed_test    # spatial-ui dashboard resolution pipeline
```
