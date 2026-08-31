#pragma once
// ─────────────────────────────────────────────────────────────────────
// DashboardFeedSystem — spatial-ui dashboard fed from domain components.
//
// Builds (once) a small VisualDocument dashboard and feeds it from the
// registry every frame:
//   • turbine mode: "SolveRun" SolveRunState → power-trace line chart +
//     stat texts (Cp, TSR, power, grid)
//   • engine mode: "SteamEngine" IndicatorRecord → indicator line chart +
//     stat texts (power, rpm, efficiency)
// Chart/text content is mutated on the document node components and tagged
// MeshDirty only when the serialized content actually changed, so the
// scene_renderer MeshSystem regenerates the widget meshes (the composer
// pattern). The document root is screen-pinned (ScreenWidgetComponent),
// handled by the scene_renderer ScreenWidgetSystem.
// ─────────────────────────────────────────────────────────────────────
#include <exd/ecs/registry.hpp>
#include <exd/ecs/system.hpp>
#include <exd/scene_renderer/document_loader.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <unordered_map>

namespace exd::sim {

class DashboardFeedSystem final : public ecs::ISystem {
public:
    explicit DashboardFeedSystem(scene_renderer::DocumentLoader& loader)
        : loader_(loader) {}

    void update(ecs::Registry& registry, double dt) override;

private:
    void ensure_dashboard(ecs::Registry& registry);
    void feed_turbine(ecs::Registry& registry);
    void feed_engine(ecs::Registry& registry);
    // Write content on a node's DocumentNodeComponent; tag MeshDirty only
    // when the content changed (cache per node id).
    void write_node(const std::string& id, const nlohmann::json& content,
                    ecs::Registry& registry);

    scene_renderer::DocumentLoader& loader_;
    bool built_ = false;
    std::unordered_map<std::string, std::string> cache_;  // node_id → content
};

} // namespace exd::sim
