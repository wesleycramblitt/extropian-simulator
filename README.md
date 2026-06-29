# Extropian Simulator

**Full scientific simulation workflow: geometry, meshing, solving, postprocessing.**

Composes all Extropian libraries with a professional UI built on RmlUi.

## Modes

| Mode | Purpose |
|------|---------|
| **Setup** | Geometry import/creation, material assignment, boundary condition specification, mesh configuration |
| **Simulate** | Solver execution with live visualization (streamlines, isosurfaces, volume rendering) |
| **Postprocess** | Data extraction, report generation, export to VTK/CGNS/CSV |

## Building

```bash
cmake -S . -B build -G Ninja
cmake --build build
./build/ExtropianSimulator
```

## License

Business Source License 1.1 — see [LICENSE](LICENSE).
Converts to Apache 2.0 on 2029-05-26.
