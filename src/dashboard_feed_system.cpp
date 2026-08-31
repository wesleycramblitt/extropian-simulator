// DashboardFeedSystem implementation — spatial-ui dashboard for the
// physics demos. Pure registry data flow: reads SolveRunState /
// IndicatorRecord (and EngineRunState), writes onto the dashboard's
// DocumentNodeComponent entities + MeshDirty tags; the scene_renderer
// MeshSystem (registered by the demo) materializes the widget meshes.
#include <exd/sim/dashboard_feed_system.hpp>

#include <exd/ecs/view.hpp>
#include <exd/scene_renderer/components/components.hpp>
#include <exd/sim/components/engine.hpp>
#include <exd/sim/components/solver_run.hpp>
#include <exd/types/visual_document.hpp>

#include <cstdio>

namespace exd::sim {

using exd::scene_renderer::DocumentNodeComponent;
using exd::scene_renderer::MeshDirty;
using exd::scene_renderer::ScreenWidgetComponent;

namespace {

exd::VisualNode text_node(std::string id, std::string text, float w, float h) {
    exd::VisualNode node;
    node.id = std::move(id);
    node.kind = exd::VisualNodeKind::Text;
    node.geometry = exd::VisualGeometrySpec{};
    node.geometry->width = w;
    node.geometry->height = h;
    node.content = nlohmann::json{{"text", std::move(text)}};
    return node;
}

exd::VisualNode chart_node(std::string id, float w, float h,
                           std::initializer_list<const char*> colors) {
    exd::VisualNode node;
    node.id = std::move(id);
    node.kind = exd::VisualNodeKind::Chart;
    node.geometry = exd::VisualGeometrySpec{};
    node.geometry->width = w;
    node.geometry->height = h;
    exd::ChartSpec spec;
    spec.grid = true;
    size_t i = 0;
    for (const char* c : colors) {
        exd::ChartSeries series;
        series.id = "s" + std::to_string(i++);
        series.color = c;
        spec.series.push_back(series);
    }
    node.chart = spec;
    node.content = nlohmann::json{{"data", nlohmann::json::array()}};
    return node;
}

} // namespace

// ── Document build (one shot) ───────────────────────────────────────

void DashboardFeedSystem::ensure_dashboard(ecs::Registry& registry) {
    if (built_) return;
    built_ = true;

    exd::VisualDocument doc;
    doc.id = "flow-dashboard";
    doc.density = "dense";
    doc.canvas.width = 600.0f;
    doc.canvas.height = 400.0f;
    doc.metadata.title = "Solver Dashboard";

    // A panel with a title, stat texts and a power chart (turbine) /
    // indicator chart (engine). Which chart gets data depends on the
    // components present in the registry (both could coexist).
    std::vector<exd::VisualNode> children;
    children.push_back(text_node("dash.title", "Solver dashboard", 560.0f, 34.0f));
    children.push_back(text_node("dash.cp", "Cp:   --", 560.0f, 26.0f));
    children.push_back(text_node("dash.tsr", "TSR:  --", 560.0f, 26.0f));
    children.push_back(text_node("dash.power", "Power: --", 560.0f, 26.0f));
    children.push_back(text_node("dash.extra", "", 560.0f, 26.0f));
    children.push_back(chart_node("dash.power_chart", 560.0f, 190.0f,
                                  {"#4cc9f0", "#f72585"}));

    exd::VisualNode root;
    root.id = "dashboard";
    root.kind = exd::VisualNodeKind::Panel;
    root.geometry = exd::VisualGeometrySpec{};
    root.geometry->width = 600.0f;
    root.geometry->height = 400.0f;
    root.layout = exd::VisualLayout{};
    root.layout->preset = "stack";   // "stack" resolves to the column strategy
    root.layout->gap = 6.0f;
    root.layout->padding = 12.0f;
    root.children = std::move(children);
    doc.nodes.push_back(std::move(root));

    const ecs::Entity root_entity = loader_.load(doc);
    if (!registry.valid(root_entity)) {
        std::printf("[Dashboard] load failed\n");
        return;
    }
    // Pin the dashboard to the top-right corner of the viewport.
    registry.emplace<ScreenWidgetComponent>(root_entity,
                                            exd::math::Vec2f{1.0f, 0.0f},
                                            exd::math::Vec2f{-16.0f, 16.0f});
    std::printf("[Dashboard] built: root=%u\n", root_entity.id);
}

// ── Node updates ────────────────────────────────────────────────────

void DashboardFeedSystem::write_node(const std::string& id,
                                     const nlohmann::json& content,
                                     ecs::Registry& registry) {
    const auto ent = loader_.entity_for(id);
    if (!ent || !registry.valid(*ent)) return;
    const std::string serialized = content.dump();
    const auto it = cache_.find(id);
    if (it != cache_.end() && it->second == serialized) return;  // unchanged
    auto& comp = registry.get<DocumentNodeComponent>(*ent);
    comp.content = content;
    registry.emplace<MeshDirty>(*ent);
    cache_[id] = serialized;
}

// ── Feeds ───────────────────────────────────────────────────────────

void DashboardFeedSystem::feed_turbine(ecs::Registry& registry) {
    registry.view<SolveRunState>().each([&](ecs::Entity, const SolveRunState& s) {
        nlohmann::json content;
        nlohmann::json data = nlohmann::json::array();
        for (int i = 0; i < s.trace_count; ++i)
            data.push_back({static_cast<double>(i), s.power_kw_trace[i]});
        content["data"] = std::move(data);
        content["points"] = false;
        content["area"] = true;
        content["ymin"] = 0.0;
        write_node("dash.power_chart", content, registry);

        write_node("dash.cp", nlohmann::json{{"text",
                    "Cp:   " + std::to_string(s.final_cp).substr(0, 6)}}, registry);
        write_node("dash.tsr", nlohmann::json{{"text",
                    "TSR:  " + std::to_string(s.final_tsr).substr(0, 6)}}, registry);
        write_node("dash.power", nlohmann::json{{"text",
                    "Power: " + std::to_string(s.final_power_kw).substr(0, 6) + " kW"}},
                   registry);
    });
}

void DashboardFeedSystem::feed_engine(ecs::Registry& registry) {
    registry.view<IndicatorRecord>().each([&](ecs::Entity, const IndicatorRecord& ind) {
        if (ind.count < 2) return;
        nlohmann::json content;
        nlohmann::json data = nlohmann::json::array();
        for (int i = 0; i < ind.count; ++i)
            data.push_back({ind.volume_litres[i], ind.pressure_kpa[i]});
        content["data"] = std::move(data);
        content["points"] = false;
        write_node("dash.power_chart", content, registry);
    });
    registry.view<EngineRunState>().each([&](ecs::Entity, const EngineRunState& st) {
        write_node("dash.cp", nlohmann::json{{"text",
                    "Power: " + std::to_string(st.mean_power_w).substr(0, 7) + " W"}},
                   registry);
        write_node("dash.tsr", nlohmann::json{{"text",
                    "Speed: " + std::to_string(
                        st.mean_omega_rad_s * 60.0 / 6.28318530718).substr(0, 6) +
                    " rpm"}}, registry);
        write_node("dash.power", nlohmann::json{{"text",
                    "Efficiency: " + std::to_string(st.efficiency).substr(0, 5)}},
                   registry);
    });
}

// ── Frame ───────────────────────────────────────────────────────────

void DashboardFeedSystem::update(ecs::Registry& registry, double dt) {
    (void)dt;
    ensure_dashboard(registry);
    feed_turbine(registry);
    feed_engine(registry);
}

} // namespace exd::sim
