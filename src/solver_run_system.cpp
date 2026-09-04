// SolverRunSystem implementation — one-shot coupled FDM3 turbine run in
// the background + flow-field visualization.
//
// Threading contract (AGENTS.md): the worker thread runs the pure physics
// call run_coupled_turbine() only — no registry, no render, no ImGui. The
// main thread polls for completion and adopts the heap payload, then writes
// components and uploads visualization meshes.
#include <exd/sim/solver_run_system.hpp>

#include <exd/ecs/view.hpp>

#include "coupled_run.hpp"

#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/renderable.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/systems/imgui_system.hpp>

#include <exd/viz/colormap.hpp>
#include <exd/viz/field.hpp>
#include <exd/viz/mesh.hpp>
#include <exd/viz/sampling.hpp>
#include <exd/viz/streamlines.hpp>

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

namespace exd::sim {

namespace {

/// True when the handle was set (i.e. not the default-constructed Entity).
bool entity_set(ecs::Entity e) {
    return e.id != std::numeric_limits<ecs::Entity::id_type>::max();
}

// ── Worker ────────────────────────────────────────────────────────────

double wall_time_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

/// Map the ECS turbine design + run config into the physics driver config.
/// Pure function (no registry, no render) so the headless test can mirror it.
/// Mirrored by solver_run_test.cpp — keep in sync.
exd::engine::presets::turbine::CoupledTurbineConfig make_run_config(
    const TurbineSpec& spec, const SolverRunConfig& cfg,
    exd::engine::ModelStatus& status) {
    using namespace exd::engine;
    namespace turbine = exd::engine::presets::turbine;
    using turbine::CoupledTurbineConfig;

    // ── Rotor definition from the ECS design ─────────────────────────
    // The demo mesh convention (TurbineSystem): section stagger =
    // pitch_deg + twist_deg·(1−span) → hub stagger = pitch+twist, tip = pitch.
    turbine::TurbineBuilderConfig b;
    b.hub_radius       = spec.hub_radius;
    b.tip_radius       = spec.radius;
    b.chord            = spec.axial_chord;
    // Twist mapping into the physics convention (measured, see solver_run_test):
    //   hub = 0 — the flat-root profile this BEM-like model resolves best
    //   tip = −pitch — sign-flipped visual stagger; −2° at the default pitch
    //            recreates the canonical recipe (Cp ≈ 0.38 on a coarse grid)
    // The visual twist ramp is aesthetic (mesh) and intentionally does not
    // enter the solve; the panel documents this.
    b.twist_hub_deg    = 0.0;
    b.twist_tip_deg    = -static_cast<double>(spec.pitch_deg);
    b.rpm              = spec.rpm;
    b.blade_count      = spec.blade_count;
    b.section_count    = 5;
    const double chord = std::max(static_cast<double>(spec.axial_chord), 0.3);
    b.leading_edge_z   = 0.5 * chord;                    // rotor plane at z = chord
    b.duct_length      = b.leading_edge_z + chord + 1.0; // validation margin
    b.shroud_radius    = 0.0;                            // open rotor
    b.default_airfoil  = "naca0012";
    const exd::geometry::TurbineDefinition def = turbine::make_turbine_definition(b, status);
    if (!status.ok) return {};

    // ── Grid: box centered on the rotor's world position ────────────
    // The demo places the visual turbine at y = 3 + radius (mirrors
    // TurbineSystem::hub_height), so the solver domain is built around
    // (0, hub_y, 0); the rotor plane sits at grid z = 0 (inflow −Z).
    CoupledTurbineConfig cc;
    const double R = std::max(static_cast<double>(spec.radius), 0.5);
    const double hub_y = 3.0 + static_cast<double>(spec.radius);
    cc.grid = turbine::default_grid_config(cfg.wind_speed, cfg.n_per_axis, 1.0, 1.0);
    cc.grid.lx = 2.0 * R * cfg.radius_margin;
    cc.grid.ly = 2.0 * (hub_y + R * cfg.radius_margin);
    cc.grid.lz = 2.0 * R * cfg.wake_length_radii;
    // Recompute the CFL-clamped dt AFTER the box override (default_grid_config
    // sized it from its nominal unit box; the driver validates stability
    // against the real dx/dy/dz).
    cc.grid.dt = 0.25 * std::min({cc.grid.dx(), cc.grid.dy(), cc.grid.dz()})
                 / std::max(static_cast<double>(cfg.wind_speed), 1e-9);
    cc.turbine = def;
    cc.element_count = 8;
    cc.rotor_inertia = 1200.0;
    // Load curve sized to the demo machine (R≈4 m, v≈6-10 m/s → P_rated ≈ 2.5 kW
    // at ω ≈ 8-12 rad/s): soft start near 0, rated Q = P/(η·ω), over-speed relief.
    // NOTE: make_generator_curve(P, η, min_omega) is NOT used here — it sizes
    // P/(η·min_omega), which blows up for min_omega « P/η and pins the rotor
    // at the low-omega clamp (measured: rotor stalled at negative ω, Cp < 0).
    exd::engine::physics::rigid_body::CurveMomentConfig generator;
    generator.omega_pts = {0.0, 2.0, 8.0, 12.0, 30.0};
    generator.torque_pts = {0.0, 40.0, 368.0, 245.0, 98.0};
    cc.generator = generator;
    // Rotor placement follows the physics fixture convention: the FDM3 box
    // spans [0, lx]×[0, ly]×[0, lz] (corner origin), so the rotor sits at
    // x = 1.8R from the wall, y = the demo's hub lift, z = 0.45·lz (45% of
    // the box upstream of the outlet, leaving 55% for the wake below it).
    // Flow-visualization entities apply the inverse offset so the field
    // lands exactly on the visual turbine (see publish_viz).
    const double m = R * cfg.radius_margin;
    const double u = 0.45 * cc.grid.lz;
    cc.rotor_origin = {m, hub_y, u};
    cc.fluid_steps_per_exchange = 12;
    cc.force_relaxation = 0.4;
    cc.smear_cells = 2.5;
    cc.max_steps = static_cast<uint64_t>(std::max(cfg.max_steps, 100));
    cc.record_history = true;
    cc.history_interval = 12;
    cc.csv_path.clear();

    // Ramp floor: the driver hard-validates ramp ≥ 10·exchange window;
    // compute the window from the CFL-clamped dt (same formula as the
    // driver) so the run never fails validation by construction.
    const double dt = cc.grid.dt > 0.0
                          ? cc.grid.dt
                          : 0.25 * std::min({cc.grid.dx(), cc.grid.dy(), cc.grid.dz()})
                                / std::max(static_cast<double>(cfg.wind_speed), 1e-9);
    const double window_s = static_cast<double>(cc.fluid_steps_per_exchange) * dt;
    cc.ramp_time_s = std::max(static_cast<double>(cfg.ramp_time_s), 10.0 * window_s + 1e-9);
    return cc;
}

/// The worker: run the coupled CFD to completion, return results only.
std::unique_ptr<SolverRunSystem::RunResult> run_coupled_worker(
    const TurbineSpec& spec, const SolverRunConfig& cfg) {
    auto payload = std::make_unique<SolverRunSystem::RunResult>();
    const double t0 = wall_time_seconds();

    exd::engine::ModelStatus status;
    auto cc = make_run_config(spec, cfg, status);
    if (!status.ok) {
        payload->error = status.error.empty() ? "invalid run configuration" : status.error;
        return payload;
    }

    const exd::engine::presets::turbine::CoupledTurbineResult res =
        exd::engine::presets::turbine::run_coupled_turbine(cc, status);
    payload->wall_seconds = wall_time_seconds() - t0;

    if (!status.ok || !res.valid) {
        payload->error = status.ok ? res.error : status.error;
        payload->valid = false;
        return payload;
    }

    payload->valid = true;
    payload->nx = res.fluid.field.nx;
    payload->ny = res.fluid.field.ny;
    payload->nz = res.fluid.field.nz;
    payload->steps_taken = res.fluid.steps_taken;
    payload->final_cp = res.final_cp;
    payload->final_tsr = res.final_tsr;
    payload->power_w = res.history.empty() ? 0.0 : res.history.back().power;
    payload->field = std::move(res.fluid.field);
    return payload;
}

// ── Visualization builders ───────────────────────────────────────────

/// Build a colored quad mesh from an RGBA-mapped slice (pixel centers on
/// field cell centers; each pixel becomes one quad).
exd::core::MeshData build_slice_quad_mesh(const exd::viz::SliceResult& slice,
                                          const std::vector<exd::math::ColorRGBA>& rgba) {
    using exd::math::Vec3f;
    exd::core::MeshData out;
    out.topology = exd::core::PrimitiveTopology::Triangles;

    const int u = slice.width();
    const int v = slice.height();
    if (u <= 0 || v <= 0) return out;

    const Vec3f du = slice.plane.u * (1.0f / static_cast<float>(u));
    const Vec3f dv = slice.plane.v * (1.0f / static_cast<float>(v));
    const Vec3f hu = du * 0.5f;
    const Vec3f hv = dv * 0.5f;

    out.vertices.reserve(static_cast<size_t>(u) * v * 4);
    out.indices.reserve(static_cast<size_t>(u) * v * 6);
    for (int j = 0; j < v; ++j) {
        for (int i = 0; i < u; ++i) {
            const size_t p = static_cast<size_t>(i) + static_cast<size_t>(u) * j;
            const Vec3f c = slice.plane.origin + du * (static_cast<float>(i) + 0.5f)
                          + dv * (static_cast<float>(j) + 0.5f);
            const exd::math::ColorRGBA& col =
                p < rgba.size() ? rgba[p] : exd::math::ColorRGBA{};
            const float alpha = slice.valid[p] ? col.a : 0.0f;

            const uint32_t base = static_cast<uint32_t>(out.vertices.size());
            const Vec3f corners[4] = {c - hu - hv, c + hu - hv, c + hu + hv, c - hu + hv};
            for (const Vec3f& corner : corners) {
                exd::core::Vertex vert;
                vert.position = corner;
                vert.color = {col.r, col.g, col.b, alpha};
                out.vertices.push_back(vert);
            }
            out.indices.insert(out.indices.end(),
                               {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    }
    for (const auto& vert : out.vertices)
        out.bounds.min = {std::min(out.bounds.min.x, vert.position.x),
                          std::min(out.bounds.min.y, vert.position.y),
                          std::min(out.bounds.min.z, vert.position.z)},
        out.bounds.max = {std::max(out.bounds.max.x, vert.position.x),
                          std::max(out.bounds.max.y, vert.position.y),
                          std::max(out.bounds.max.z, vert.position.z)};
    return out;
}

} // namespace

// ── Entities ─────────────────────────────────────────────────────────

void SolverRunSystem::ensure_entities(ecs::Registry& registry) {
    if (entities_ready_) return;

    run_ = registry.create("SolverRun");
    registry.emplace<SolverRunConfig>(run_);
    registry.emplace<SolveRunState>(run_);

    flow_slice_ = registry.create("FlowSlice");
    registry.emplace<render::Transform>(flow_slice_);
    registry.emplace<render::RenderableComponent>(flow_slice_, 0u);
    registry.emplace<render::RenderTechnique_Mirror>(flow_slice_);

    flow_lines_ = registry.create("FlowStreamlines");
    registry.emplace<render::Transform>(flow_lines_);
    registry.emplace<render::RenderableComponent>(flow_lines_, 0u);
    registry.emplace<render::RenderTechnique_Mirror>(flow_lines_);

    if (!panel_added_) {
        auto panel = registry.create("SolveRunPanel");
        registry.emplace<render::ImGuiPanelComponent>(panel, "Solver Run",
            [this] { draw_panel(); });
        panel_added_ = true;
    }

    entities_ready_ = true;
}

void SolverRunSystem::find_turbine(ecs::Registry& registry) {
    if (entity_set(turbine_) && registry.valid(turbine_)) return;
    registry.view<TurbineSpec>().each(
        [this](ecs::Entity e, const TurbineSpec&) { if (!entity_set(turbine_)) turbine_ = e; });
}

// ── Solve ────────────────────────────────────────────────────────────

void SolverRunSystem::solve() {
    if (!reg_ || solving()) return;
    if (!entity_set(turbine_) || !reg_->valid(turbine_)) return;
    if (auto* spec = reg_->try_get<TurbineSpec>(turbine_)) {
        const SolverRunConfig cfg = reg_->get<SolverRunConfig>(run_);
        // snapshot to the worker; the payload returns through the future
        const TurbineSpec design = *spec;
        const SolverRunConfig run_cfg = cfg;
        future_ = std::async(std::launch::async, [design, run_cfg] {
            auto payload = std::make_unique<RunResult>();
            exd::engine::physics::fluid::fdm3::FDM3FieldData field;
            const impl::CoupledRunOutcome outcome =
                impl::run_coupled_eval(design, run_cfg.wind_speed, run_cfg.n_per_axis,
                                       run_cfg.max_steps, run_cfg.ramp_time_s,
                                       run_cfg.radius_margin, run_cfg.wake_length_radii,
                                       &field);
            payload->valid = outcome.valid;
            payload->error = outcome.error;
            payload->nx = outcome.nx; payload->ny = outcome.ny; payload->nz = outcome.nz;
            payload->steps_taken = outcome.steps_taken;
            payload->final_cp = outcome.final_cp;
            payload->final_tsr = outcome.final_tsr;
            payload->power_w = outcome.power_w;
            payload->wall_seconds = outcome.wall_seconds;
            payload->power_w_trace = std::move(outcome.power_w_trace);
            payload->field = std::move(field);
            return payload;
        });
        worker_running_.store(true, std::memory_order_relaxed);
        auto& state = reg_->get<SolveRunState>(run_);
        state.status = SolveRunState::Running;
        std::printf("[SolverRun] coupled FDM3 run started: R=%.1f m, v=%.1f m/s, "
                    "%d³ grid, ≤%d steps\n",
                    spec->radius, cfg.wind_speed, cfg.n_per_axis, cfg.max_steps);
    }
}

// ── Frame ────────────────────────────────────────────────────────────

void SolverRunSystem::update(ecs::Registry& registry, double dt) {
    (void)dt;
    reg_ = &registry;
    ensure_entities(registry);
    find_turbine(registry);
    poll_worker(registry);
}

void SolverRunSystem::poll_worker(ecs::Registry& registry) {
    if (!solving()) return;

    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;

    std::unique_ptr<RunResult> payload;
    try {
        payload = future_.get();
    } catch (...) {
        payload = std::make_unique<RunResult>();
        payload->error = "worker exception";
    }
    worker_running_.store(false, std::memory_order_relaxed);
    result_ = std::move(payload);

    // Main thread only from here on.
    auto& state = registry.get<SolveRunState>(run_);

    if (!result_ || !result_->valid) {
        state.status = SolveRunState::Failed;
        state.steps_taken = 0;
        last_error_ = result_ ? result_->error : "no result";
        std::printf("[SolverRun] failed: %s\n", last_error_.c_str());
        return;
    }

    state.status = SolveRunState::Done;
    state.grid_nx = result_->nx;
    state.grid_ny = result_->ny;
    state.grid_nz = result_->nz;
    state.steps_taken = result_->steps_taken;
    state.final_cp = result_->final_cp;
    state.final_tsr = result_->final_tsr;
    state.final_power_kw = result_->power_w * 1e-3;
    state.wall_seconds = result_->wall_seconds;
    last_error_ = "none";

    // Decimate the power trace into the POD component for dashboards.
    const size_t n = result_->power_w_trace.size();
    const int stride = n > SolveRunState::kTraceSamples
                           ? static_cast<int>(n / SolveRunState::kTraceSamples)
                           : 1;
    state.trace_count = 0;
    for (size_t i = 0; i < n && state.trace_count < SolveRunState::kTraceSamples;
         i += static_cast<size_t>(std::max(stride, 1))) {
        state.power_kw_trace[state.trace_count] =
            static_cast<float>(result_->power_w_trace[i] * 1e-3);
        ++state.trace_count;
    }

    std::printf("[SolverRun] done: %d steps, Cp=%.3f TSR=%.2f P=%.1f kW "
                "(%d³ grid, %.1f s wall)\n",
                state.steps_taken, state.final_cp, state.final_tsr,
                state.final_power_kw, state.grid_nx, state.wall_seconds);

    publish_viz(registry);
}

// ── Visualization ────────────────────────────────────────────────────

void SolverRunSystem::publish_viz(ecs::Registry& registry) {
    if (!result_ || !result_->valid || result_->field.u.empty()) return;
    const SolverRunConfig& cfg = registry.get<SolverRunConfig>(run_);
    const exd::engine::physics::fluid::fdm3::FDM3FieldData& f = result_->field;

    // ── Uniform grid + fields from the solver's cell-centered arrays ──
    exd::viz::UniformGrid grid;
    grid.nx = f.nx; grid.ny = f.ny; grid.nz = f.nz;
    grid.origin = {static_cast<float>(f.x.empty() ? 0.0 : f.x.front()),
                   static_cast<float>(f.y.empty() ? 0.0 : f.y.front()),
                   static_cast<float>(f.z.empty() ? 0.0 : f.z.front())};
    grid.spacing = {static_cast<float>((f.x.size() > 1 ? f.x[1] - f.x[0] : 1.0)),
                    static_cast<float>((f.y.size() > 1 ? f.y[1] - f.y[0] : 1.0)),
                    static_cast<float>((f.z.size() > 1 ? f.z[1] - f.z[0] : 1.0))};

    exd::viz::VectorField3D vel(grid);
    exd::viz::ScalarField3D speed(grid);
    float vmin = std::numeric_limits<float>::max();
    float vmax = 0.0f;
    for (int k = 0; k < f.nz; ++k) {
        for (int j = 0; j < f.ny; ++j) {
            for (int i = 0; i < f.nx; ++i) {
                const size_t idx = f.index(i, j, k);
                const float u = static_cast<float>(f.u[idx]);
                const float v = static_cast<float>(f.v[idx]);
                const float w = static_cast<float>(f.w[idx]);
                vel.at(i, j, k) = {u, v, w};
                const float mag = std::sqrt(u * u + v * v + w * w);
                speed.at(i, j, k) = mag;
                vmin = std::min(vmin, mag);
                vmax = std::max(vmax, mag);
            }
        }
    }

    const float vrange_max = std::max(vmax, 1.0f);
    const exd::viz::TransferFunction tf = exd::viz::TransferFunction::viridis();
    exd::viz::ValueRange range{0.0f, vrange_max, true};

    // ── Axial slice through the rotor plane (grid z = rotor z) ──────
    const int layer_z = std::clamp(static_cast<int>(0.45f * static_cast<float>(f.nz)),
                                   0, std::max(f.nz - 1, 0));
    const exd::viz::SlicePlane plane = exd::viz::axis_aligned_slice(grid, 2, layer_z);
    const exd::viz::SliceResult slice = exd::viz::extract_slice(speed, plane);
    const std::vector<exd::math::ColorRGBA> rgba =
        exd::viz::map_slice(slice, tf, range, /*mask_invalid=*/true);
    exd::core::MeshData slice_mesh = build_slice_quad_mesh(slice, rgba);

    if (!slice_mesh.vertices.empty()) {
        if (slice_handle_ != 0) ctx_.mesh_manager.destroy(slice_handle_);
        slice_handle_ = ctx_.mesh_manager.create(slice_mesh);
        registry.get<render::RenderableComponent>(flow_slice_).mesh = slice_handle_;
        std::printf("[SolverRun] slice viz: %zu verts (layer %d)\n",
                    slice_mesh.vertices.size(), layer_z);
    }

    // ── Streamlines seeded upstream (+Z), raked across the rotor ────
    const float R = static_cast<float>(
        std::max(registry.get<TurbineSpec>(turbine_).radius, 0.5f));
    const float rotor_x = static_cast<float>(cfg.radius_margin) * R;
    const float rotor_y = 3.0f + R;                 // mirrors TurbineSystem lift
    const float rotor_z = static_cast<float>(f.z[layer_z]); // rotor-plane layer
    const float rake = 0.8f * R;                    // seeding extent
    exd::viz::StreamlineSeedSpec seeds;
    seeds.origin = {rotor_x - rake, rotor_y - rake, rotor_z + 1.2f * R};
    seeds.spacing = {rake * 0.5f, rake * 0.5f, 0.0f};
    seeds.nx = 5; seeds.ny = 5; seeds.nz = 1;

    exd::viz::StreamlineParams params;
    params.step_size = 0.0f;                 // cell_size_scale default
    params.max_steps = 2500;

    std::vector<exd::viz::Polyline3> lines;
    exd::viz::advect_streamlines_from_seeds_into(
        vel, exd::viz::generate_seeds(vel.grid(), seeds), params, lines);

    exd::core::MeshData lines_mesh =
        exd::viz::polylines_to_mesh(lines, {1.0f, 1.0f, 1.0f});
    std::vector<exd::math::ColorRGB> line_colors;
    exd::viz::color_mesh_by_field(speed, lines_mesh, tf, range, line_colors);
    if (line_colors.size() == lines_mesh.vertices.size())
        exd::viz::apply_vertex_colors(lines_mesh, line_colors, 1.0f);

    if (!lines_mesh.vertices.empty()) {
        if (lines_handle_ != 0) ctx_.mesh_manager.destroy(lines_handle_);
        lines_handle_ = ctx_.mesh_manager.create(lines_mesh);
        registry.get<render::RenderableComponent>(flow_lines_).mesh = lines_handle_;
        std::printf("[SolverRun] streamline viz: %zu polys, %zu verts\n",
                    lines.size(), lines_mesh.vertices.size());
    }

    // Grid coords → world: the solver box spans [0, lx]×[0, ly]×[0, lz] with
    // the rotor at (m, hub_y, 0.45·lz); the visual turbine sits at the world
    // origin of its own transform. Offset the flow entities by −(m, 0, −u)
    // so the field lands exactly on the machine.
    const exd::math::Vec3f flow_offset = {
        -static_cast<float>(cfg.radius_margin) * R, 0.0f, -rotor_z};
    registry.get<render::Transform>(flow_slice_).position = flow_offset;
    registry.get<render::Transform>(flow_lines_).position = flow_offset;
}

// ── Panel ────────────────────────────────────────────────────────────

void SolverRunSystem::draw_panel() {
    if (!reg_ || !entity_set(run_) || !reg_->valid(run_)) return;
    auto& cfg = reg_->get<SolverRunConfig>(run_);
    const auto& state = reg_->get<SolveRunState>(run_);

    ImGui::Text("Coupled FDM3 turbine run (real CFD)");
    ImGui::Separator();
    ImGui::SliderFloat("Wind speed [m/s]", &cfg.wind_speed, 3.0f, 20.0f, "%.1f");
    ImGui::SliderInt("Grid cells / axis", &cfg.n_per_axis, 12, 32);
    ImGui::SliderInt("Max fluid steps", &cfg.max_steps, 300, 12000);
    ImGui::SliderFloat("Ramp time [s]", &cfg.ramp_time_s, 0.5f, 8.0f, "%.1f");
    ImGui::Separator();

    const bool idle = !solving();
    if (!idle) ImGui::BeginDisabled();
    if (ImGui::Button("Solve Current Design")) solve();
    if (!idle) ImGui::EndDisabled();

    if (solving()) {
        ImGui::Text("Running in background...");
    } else {
        const char* status = state.status == SolveRunState::Done    ? "Done"
                             : state.status == SolveRunState::Failed ? "Failed"
                                                                     : "Idle";
        ImGui::Text("Status: %s", status);
        if (state.status == SolveRunState::Failed)
            ImGui::TextWrapped("Error: %s", last_error_.c_str());
        if (state.status == SolveRunState::Done) {
            ImGui::Text("Cp: %.3f   TSR: %.2f", state.final_cp, state.final_tsr);
            ImGui::Text("Shaft power: %.1f kW", state.final_power_kw);
            ImGui::Text("%d steps on a %dx%dx%d grid (%.1f s)",
                        state.steps_taken, state.grid_nx, state.grid_ny, state.grid_nz,
                        state.wall_seconds);
        }
    }

    ImGui::Separator();
    ImGui::TextWrapped(
        "Solves the current turbine design with the actuator-disk-corrected "
        "FDM3 CFD solver (extropian-physics). The run executes on a worker "
        "thread; slice + streamlines of the final flow field appear in the "
        "viewport. Tune the Turbine panel first, then solve.");
}

} // namespace exd::sim
