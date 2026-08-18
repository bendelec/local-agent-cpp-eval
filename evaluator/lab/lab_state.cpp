#include "lab_state.hpp"

#include <utility>

namespace vwmini_lab {
namespace {

[[nodiscard]] vwmini::Polygon triangle(vwmini::Vec2 a, vwmini::Vec2 b, vwmini::Vec2 c)
{
    return {{a, b, c}};
}

[[nodiscard]] std::vector<vwmini::Polygon> default_triangles()
{
    // An L-shaped space: a useful default for route and containment diagnostics.
    return {
        triangle({0.0f, 0.0f}, {4.0f, 0.0f}, {4.0f, 4.0f}),
        triangle({0.0f, 0.0f}, {4.0f, 4.0f}, {0.0f, 4.0f}),
        triangle({0.0f, 4.0f}, {4.0f, 4.0f}, {4.0f, 8.0f}),
        triangle({0.0f, 4.0f}, {4.0f, 8.0f}, {0.0f, 8.0f}),
        triangle({4.0f, 0.0f}, {8.0f, 0.0f}, {8.0f, 4.0f}),
        triangle({4.0f, 0.0f}, {8.0f, 4.0f}, {4.0f, 4.0f}),
    };
}

} // namespace

LabState::LabState()
{
    reset_default_mesh();
}

void LabState::reset_default_mesh()
{
    m_triangles = default_triangles();
    static_cast<void>(rebuild_mesh());
}

bool LabState::rebuild_mesh()
{
    auto mesh = vwmini::NavMesh::create(m_triangles);
    if (!mesh) {
        m_triangles = m_last_valid_triangles;
        m_diagnostic = mesh.error().message;
        return false;
    }

    m_last_valid_triangles = m_triangles;
    m_mesh = std::move(*mesh);
    m_simulation = std::make_unique<vwmini::Simulation>(*m_mesh);
    m_agent_ids.clear();
    m_diagnostic = "mesh accepted; agents reset";
    return true;
}

void LabState::advance(float seconds)
{
    if (!m_simulation) {
        return;
    }
    if (const auto result = m_simulation->step(seconds); !result) {
        m_diagnostic = result.error().message;
    }
}

bool LabState::add_agent(vwmini::Vec2 position, vwmini::Vec2 goal)
{
    if (!m_simulation) {
        return false;
    }
    vwmini::AgentConfig config;
    config.position = position;
    config.goal = goal;
    const auto added = m_simulation->add_agent(config);
    if (!added) {
        m_diagnostic = added.error().message;
        return false;
    }
    m_agent_ids.push_back(*added);
    m_diagnostic = "agent added";
    return true;
}

bool LabState::set_goal(vwmini::AgentId id, vwmini::Vec2 goal)
{
    if (!m_simulation) {
        return false;
    }
    if (const auto result = m_simulation->set_goal(id, goal); !result) {
        m_diagnostic = result.error().message;
        return false;
    }
    m_diagnostic = "goal updated";
    return true;
}

bool LabState::remove_agent(vwmini::AgentId id)
{
    if (!m_simulation) {
        return false;
    }
    if (const auto result = m_simulation->remove_agent(id); !result) {
        m_diagnostic = result.error().message;
        return false;
    }
    std::erase(m_agent_ids, id);
    m_diagnostic = "agent removed";
    return true;
}

std::optional<vwmini::Path> LabState::route_preview(vwmini::AgentId id) const
{
    if (!m_mesh || !m_simulation) {
        return std::nullopt;
    }
    const auto state = m_simulation->agent(id);
    if (!state || !state->goal || state->status == vwmini::AgentStatus::NoPath) {
        return std::nullopt;
    }
    const auto path = vwmini::find_path(*m_mesh, state->position, *state->goal);
    return path ? std::optional<vwmini::Path>{*path} : std::nullopt;
}

std::optional<vwmini::AgentId> LabState::nearest_agent(vwmini::Vec2 point, float radius) const
{
    if (!m_simulation) {
        return std::nullopt;
    }
    const float radius_squared = radius * radius;
    std::optional<vwmini::AgentId> nearest;
    float nearest_squared = radius_squared;
    for (const vwmini::AgentId id : m_agent_ids) {
        const auto state = m_simulation->agent(id);
        if (!state) {
            continue;
        }
        const vwmini::Vec2 delta = state->position - point;
        const float distance_squared = vwmini::dot(delta, delta);
        if (distance_squared <= nearest_squared) {
            nearest_squared = distance_squared;
            nearest = id;
        }
    }
    return nearest;
}

std::vector<DisplayAgent> LabState::agents() const
{
    std::vector<DisplayAgent> result;
    if (!m_simulation) {
        return result;
    }
    for (const vwmini::AgentId id : m_agent_ids) {
        if (const auto state = m_simulation->agent(id)) {
            result.push_back({id, *state});
        }
    }
    return result;
}

const std::vector<vwmini::Polygon>& LabState::triangles() const noexcept
{
    return m_triangles;
}

std::vector<vwmini::Polygon>& LabState::triangles() noexcept
{
    return m_triangles;
}

const std::string& LabState::diagnostic() const noexcept
{
    return m_diagnostic;
}

} // namespace vwmini_lab
