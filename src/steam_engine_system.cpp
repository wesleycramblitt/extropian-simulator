// SteamEngineSystem implementation — steam engine meshing + simulation.
//
// Threading contract (AGENTS.md): simulate_engine() runs on a worker; the
// main thread adopts the heap outcome and writes components / uploads the
// machine mesh (regenerated per crank angle, throttled by a dirty step).
#include <exd/sim/steam_engine_system.hpp>

#include "engine_run.hpp"

#include <exd/geometry/steam_engine.hpp>
#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/renderable.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/systems/imgui_system.hpp>

#include <imgui.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <limits>

namespace exd::sim {

// Worker channel (header pimpl): holds the future + adopted outcome.
struct SteamEngineSystem::Worker {
    std::future<std::unique_ptr<impl::EngineRunOutcome>> future;
    std::atomic<bool> busy{false};
    std::unique_ptr<impl::EngineRunOutcome> result;
};

SteamEngineSystem::SteamEngineSystem(render::GraphicsContext& ctx)
    : ctx_(ctx), worker_(std::make_unique<Worker>()) {}

SteamEngineSystem::~SteamEngineSystem() = default;

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRebuildStepDeg = 4.0;   // mesh rebuild quantization
constexpr double kIdleSpinDegPerSec = 90.0;
constexpr double kAnimateSpeedScale = 0.05;  // 5% of real mean ω while watching
} // namespace

// ── Entities / mesh ─────────────────────────────────────────────────

void SteamEngineSystem::ensure_entities(ecs::Registry& registry) {
    if (entities_ready_) return;

    entity_ = registry.create("SteamEngine");
    registry.emplace<render::Transform>(entity_);
    registry.emplace<render::RenderableComponent>(entity_, 0u);
    registry.emplace<render::RenderTechnique_Mirror>(entity_);
    registry.emplace<EngineSpec>(entity_);
    registry.emplace<EngineRunState>(entity_);
    registry.emplace<IndicatorRecord>(entity_);
    spec_snapshot_ = EngineSpec{};
    rebuild_mesh(registry);
    last_spec_ = registry.get<EngineSpec>(entity_);

    if (!panel_added_) {
        panel_ = registry.create("SteamEnginePanel");
        registry.emplace<render::ImGuiPanelComponent>(panel_, "Steam Engine",
            [this] { draw_panel(); });
        panel_added_ = true;
    }
    entities_ready_ = true;
}

void SteamEngineSystem::rebuild_mesh(ecs::Registry& registry) {
    const EngineSpec& e = registry.get<EngineSpec>(entity_);

    // Visual definition: physics geometry (crank/rod/bore) + default
    // machine proportions sized from the bore.
    exd::geometry::SteamEngineDefinition def;
    def.crank_angle_deg = static_cast<float>(crank_deg_);
    def.crank_radius    = e.crank_radius;
    def.conrod_length   = std::max(e.rod_length, e.crank_radius * 1.5f);
    def.piston_radius   = e.bore * 0.5f;
    def.cylinder_bore_radius = e.bore * 0.55f;
    def.cylinder_outer_radius = e.bore * 0.80f;
    def.cylinder_length = std::max(e.bore * 3.0f, def.conrod_length * 0.7f);
    def.crank_center_x  = def.cylinder_length * 0.75f;

    const exd::core::MeshData mesh = exd::geometry::generate_steam_engine_mesh(def);
    if (mesh.vertices.empty()) {
        std::printf("[SteamEngine] generate_steam_engine_mesh empty\n");
        return;
    }

    const uint32_t handle = ctx_.mesh_manager.create(mesh);
    if (mesh_handle_ != 0) ctx_.mesh_manager.destroy(mesh_handle_);
    mesh_handle_ = handle;
    registry.get<render::RenderableComponent>(entity_).mesh = handle;
    last_rebuilt_deg_ = crank_deg_;
    std::printf("[SteamEngine] mesh rebuilt: %zu verts at θ=%.0f°\n",
                mesh.vertices.size(), crank_deg_);
}

// ── Run ─────────────────────────────────────────────────────────────

bool SteamEngineSystem::running() const {
    return worker_->busy.load(std::memory_order_relaxed);
}

void SteamEngineSystem::start_run() {
    if (!reg_ || running()) return;
    spec_snapshot_ = reg_->get<EngineSpec>(entity_);
    worker_->future = std::async(std::launch::async, [this] {
        return std::make_unique<impl::EngineRunOutcome>(
            impl::run_engine_eval(spec_snapshot_));
    });
    worker_->busy.store(true, std::memory_order_relaxed);
    auto& state = reg_->get<EngineRunState>(entity_);
    state.status = EngineRunState::Running;
    std::printf("[SteamEngine] run started: p_boiler=%.2f MPa, cutoff=%.0f°, "
                "R=%.3f m, bore=%.3f m\n",
                spec_snapshot_.p_boiler * 1e-6, spec_snapshot_.cutoff_deg,
                spec_snapshot_.crank_radius, spec_snapshot_.bore);
}

void SteamEngineSystem::poll_worker(ecs::Registry& registry) {
    if (!running()) return;
    if (worker_->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return;

    std::unique_ptr<impl::EngineRunOutcome> res;
    try {
        res = worker_->future.get();
    } catch (...) {
        res = std::make_unique<impl::EngineRunOutcome>();
        res->error = "worker exception";
    }
    worker_->busy.store(false, std::memory_order_relaxed);
    worker_->result = std::move(res);

    auto& state = registry.get<EngineRunState>(entity_);
    auto& ind = registry.get<IndicatorRecord>(entity_);

    if (!worker_->result || !worker_->result->valid) {
        state.status = EngineRunState::Failed;
        last_error_ = worker_->result ? worker_->result->error : "no result";
        std::printf("[SteamEngine] failed: %s\n", last_error_.c_str());
        return;
    }

    state.status = EngineRunState::Done;
    state.total_time_s = worker_->result->total_time_s;
    state.mean_power_w = worker_->result->mean_power_w;
    state.mean_omega_rad_s = worker_->result->mean_omega_rad_s;
    state.cycles = worker_->result->cycles;
    state.efficiency = worker_->result->efficiency;
    state.wall_seconds = worker_->result->wall_seconds;
    state.steps_taken = static_cast<int>(worker_->result->cycles);  // proxy
    last_error_ = "none";

    // Rebuild the indicator record (fixed-size POD component).
    ind.count = 0;
    const size_t n = worker_->result->indicator.size();
    const int stride = n > static_cast<size_t>(IndicatorRecord::kMaxSamples)
                           ? static_cast<int>(n / IndicatorRecord::kMaxSamples)
                           : 1;
    if (n > 0) {
        for (size_t i = 0; i < n && ind.count < IndicatorRecord::kMaxSamples;
             i += static_cast<size_t>(std::max(stride, 1))) {
            ind.crank_deg[ind.count] = worker_->result->indicator[i].theta_deg;
            ind.pressure_kpa[ind.count] = worker_->result->indicator[i].p_kpa;
            ind.volume_litres[ind.count] = worker_->result->indicator[i].v_litres;
            ++ind.count;
        }
    }

    std::printf("[SteamEngine] done: %.1f s sim, %.0f cycles, P=%.1f W, "
                "ω=%.1f rad/s, η=%.3f (%.2f s wall)\n",
                state.total_time_s, state.cycles, state.mean_power_w,
                state.mean_omega_rad_s, state.efficiency, state.wall_seconds);
}

// ── Frame ───────────────────────────────────────────────────────────

void SteamEngineSystem::update(ecs::Registry& registry, double dt) {
    reg_ = &registry;
    ensure_entities(registry);
    poll_worker(registry);

    // Spec dirty check → full rebuild at the current crank angle.
    const EngineSpec& spec = registry.get<EngineSpec>(entity_);
    if (!(spec.crank_radius == last_spec_.crank_radius &&
          spec.rod_length == last_spec_.rod_length &&
          spec.bore == last_spec_.bore)) {
        rebuild_mesh(registry);
        last_spec_ = spec;
    }

    // Animation: advance the crank; rebuild on the quantization step.
    double deg_per_s = kIdleSpinDegPerSec;
    const auto& state = registry.get<EngineRunState>(entity_);
    if (state.status == EngineRunState::Done)
        deg_per_s = state.mean_omega_rad_s / kDeg2Rad * kAnimateSpeedScale;
    crank_deg_ += deg_per_s * static_cast<double>(dt);
    while (crank_deg_ > 360.0) crank_deg_ -= 360.0;
    while (crank_deg_ < 0.0) crank_deg_ += 360.0;

    double delta = crank_deg_ - last_rebuilt_deg_;
    while (delta > 360.0) delta -= 360.0;
    while (delta < -360.0) delta += 360.0;
    if (std::fabs(delta) >= kRebuildStepDeg)
        rebuild_mesh(registry);
}

// ── Panel ───────────────────────────────────────────────────────────

void SteamEngineSystem::draw_panel() {
    if (!reg_) return;
    auto& e = reg_->get<EngineSpec>(entity_);
    const auto& state = reg_->get<EngineRunState>(entity_);

    ImGui::Text("Steam engine — 0D Rankine-lite simulator");
    ImGui::Separator();
    ImGui::SliderFloat("Crank radius [m]", &e.crank_radius, 0.02f, 0.15f, "%.3f");
    ImGui::SliderFloat("Conrod length [m]", &e.rod_length, 0.08f, 0.40f, "%.3f");
    ImGui::SliderFloat("Bore [m]", &e.bore, 0.05f, 0.20f, "%.3f");
    ImGui::SliderFloat("Flywheel inertia", &e.flywheel_inertia, 0.005f, 0.10f, "%.4f");
    ImGui::Separator();
    ImGui::SliderFloat("Boiler pressure [MPa]", &e.p_boiler, 0.2f, 2.0f, "%.2f");
    ImGui::SliderFloat("Condenser pressure [kPa]", &e.p_condenser, 5.0f, 100.0f, "%.0f");
    ImGui::SliderFloat("Cutoff [deg]", &e.cutoff_deg, 10.0f, 120.0f, "%.0f");
    ImGui::SliderFloat("Steam quality", &e.steam_quality, 0.80f, 1.0f, "%.2f");
    ImGui::Separator();

    if (running()) {
        ImGui::Text("Simulating in background...");
    } else {
        if (ImGui::Button("Run Engine")) start_run();
        const char* status = state.status == EngineRunState::Done    ? "Done"
                             : state.status == EngineRunState::Failed ? "Failed"
                                                                      : "Idle";
        ImGui::Text("Status: %s", status);
        if (state.status == EngineRunState::Failed)
            ImGui::TextWrapped("Error: %s", last_error_.c_str());
        if (state.status == EngineRunState::Done) {
            ImGui::Text("Mean power: %.1f W", state.mean_power_w);
            ImGui::Text("Mean speed: %.0f rpm (%.1f rad/s)",
                        state.mean_omega_rad_s * 60.0 / (2.0 * 3.14159265),
                        state.mean_omega_rad_s);
            ImGui::Text("Efficiency: %.1f %%", state.efficiency * 100.0);
            ImGui::Text("%.0f cycles in %.1f s sim (%.1f s wall)",
                        state.cycles, state.total_time_s, state.wall_seconds);
        }
    }

    // Interim indicator strip (replaced by the spatial-ui diagram in M5).
    const auto& ind = reg_->get<IndicatorRecord>(entity_);
    if (ind.count > 1) {
        ImGui::Separator();
        ImGui::Text("Indicator: cylinder pressure 0..360°");
        ImGui::PlotLines("p [kPa]", ind.pressure_kpa, ind.count,
                         0, nullptr,
                         *std::min_element(ind.pressure_kpa, ind.pressure_kpa + ind.count),
                         *std::max_element(ind.pressure_kpa, ind.pressure_kpa + ind.count),
                         ImVec2(0.0f, 80.0f));
    }

    ImGui::Separator();
    ImGui::TextWrapped(
        "Single-cylinder steam engine: constant-pressure admission to the "
        "cutoff, wet-steam polytropic expansion, exhaust to the condenser. "
        "Run simulates ~10 s of shaft motion; the crank animates at a scaled "
        "speed afterward. Optimization (engine mode) varies boiler pressure, "
        "cutoff, crank radius and bore.");
}

} // namespace exd::sim
