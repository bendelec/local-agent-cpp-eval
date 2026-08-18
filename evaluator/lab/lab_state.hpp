#pragma once

#include <vwmini/simulation.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace vwmini_lab {

struct DisplayAgent {
    vwmini::AgentId id{};
    vwmini::AgentState state{};
};

class LabState {
public:
    LabState();

    void reset_default_mesh();
    [[nodiscard]] bool rebuild_mesh();
    void advance(float seconds);

    [[nodiscard]] bool add_agent(vwmini::Vec2 position, vwmini::Vec2 goal);
    [[nodiscard]] bool set_goal(vwmini::AgentId id, vwmini::Vec2 goal);
    [[nodiscard]] bool remove_agent(vwmini::AgentId id);
    /** Public-API route preview from the agent's current position to its goal. */
    [[nodiscard]] std::optional<vwmini::Path> route_preview(vwmini::AgentId id) const;

    [[nodiscard]] std::optional<vwmini::AgentId> nearest_agent(vwmini::Vec2 point,
                                                                 float radius) const;
    [[nodiscard]] std::vector<DisplayAgent> agents() const;
    [[nodiscard]] const std::vector<vwmini::Polygon>& triangles() const noexcept;
    [[nodiscard]] std::vector<vwmini::Polygon>& triangles() noexcept;
    [[nodiscard]] const std::string& diagnostic() const noexcept;

private:
    std::vector<vwmini::Polygon> m_triangles;
    std::vector<vwmini::Polygon> m_last_valid_triangles;
    // Retained only to service diagnostic path previews through the public API.
    std::optional<vwmini::NavMesh> m_mesh;
    std::unique_ptr<vwmini::Simulation> m_simulation;
    std::vector<vwmini::AgentId> m_agent_ids;
    std::string m_diagnostic;
};

} // namespace vwmini_lab
