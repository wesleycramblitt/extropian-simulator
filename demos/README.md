# Demos

One executable per registration example. All demos subclass `DemoApp` and
only **register systems** — every system lives in the `exd::sim` library.

| Demo | What it shows | Systems registered |
|---|---|---|
| `extropian-sim-shape-workshop` (default) | primitive-shape gallery from extropian-geometry recipes (2D + 3D), live parametric control, and 3D gizmo editing (translate/rotate/scale on geometry gizmo meshes) | ShapeWorkshopSystem, Gizmo3DSystem + Picker/Selection (render tooling), CameraModeSystem |
| `extropian-sim-camera-modes` | the unified mobile camera: FPS / orbit / ground-walk / orthographic-2D editing + runtime FOV | ShapeWorkshopSystem (small starter gallery), CameraModeSystem |
| `extropian-sim-optimize` | minimal multi-system composition: analytic-objective CMA-ES optimization driving a live turbine design | OptimizationSystem (Analytic), TurbineSystem |
| `extropian-sim-turbine-solver` | real meshing + CFD: full coupled FDM3 solver runs in the background; the flow field is published as a speed slice + streamlines; CMA-ES with one short coupled-CFD run **per candidate**; spatial-ui dashboard | OptimizationSystem (CoupledCfd), SolverRunSystem, TurbineSystem, DashboardFeedSystem + scene_renderer UI pipeline |
| `extropian-sim-steam-engine` | steam engine: parametric machine meshing animated per crank angle, 0D Rankine-lite solver runs, CMA-ES over boiler pressure/cutoff/geometry, indicator dashboard | OptimizationSystem (EngineSim), SteamEngineSystem, DashboardFeedSystem + scene_renderer UI pipeline |

## Run

```sh
./build.sh                    # builds everything
./run.sh                      # default demo (extropian-sim-shape-workshop)
./run.sh extropian-sim-optimize
./run.sh extropian-sim-turbine-solver
./run.sh extropian-sim-steam-engine
./run.sh extropian-sim-camera-modes
```

Shape workshop controls: click a shape to select; 1/2/3 switch
translate/rotate/scale gizmo; left-drag spins the orbit camera (or drag the
gizmo), scroll zooms, `Z` toggles fly camera / UI mode.

Camera modes controls: keys 1/2/3/4 = FPS / Orbit / Walk / Ortho2D; the panel
has mode buttons, a FOV slider, and per-mode parameters.

`Z` toggles fly camera / UI mode. The physics demos accept `--auto-run`
(start optimization + a solver run ~0.5 s after boot — a smoke test).

## Copying a demo

`demos/<name>/main.cpp` is the full application; the only hook is
`register_sim_systems(graph)` plus optional `on_update` logic. Add the
executable + asset-copy entry in `demos/CMakeLists.txt`.

## Headless tests

```sh
./build/optimization_test      # CMA-ES objective (analytic reference)
./build/shape_workshop_test    # shape recipe dispatch (spec → mesh)
./build/solver_run_test        # coupled-run recipe: valid + deterministic
./build/engine_run_test        # engine-run recipe: valid + deterministic + sensitivity
./build/dashboard_feed_test    # spatial-ui dashboard resolution pipeline
```
