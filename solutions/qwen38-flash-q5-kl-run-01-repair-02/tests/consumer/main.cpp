#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <cmath>
#include <limits>
#include <vector>

int main()
{
    using namespace vwmini;

    std::vector<Polygon> triangles;
    triangles.push_back(Polygon{{Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}, Vec2{1.0f, 1.0f}}});
    triangles.push_back(Polygon{{Vec2{0.0f, 0.0f}, Vec2{1.0f, 1.0f}, Vec2{0.0f, 4.0f}}});
    const auto mesh = NavMesh::create(std::move(triangles));
    if (!mesh.has_value())
    {
        return 1;
    }

    Simulation sim{NavMesh(*mesh)};
    AgentConfig config;
    config.position = Vec2{0.25f, 0.25f};
    config.radius = 0.2f;
    config.max_speed = 1.0f;
    config.goal = Vec2{0.6f, 0.6f};
    const auto id = sim.add_agent(config);
    if (!id.has_value())
    {
        return 2;
    }
    for (int index = 0; index < 40; ++index)
    {
        if (!sim.step(0.1f).has_value())
        {
            return 3;
        }
    }
    if (sim.agent(AgentId{999u}).has_value())
    {
        return 3;
    }
    // Numeric policy, visible downstream: a triangle whose edge products overflow `float` is
    // still accepted, still answers containment correctly, and still routes.
    const auto huge_mesh = NavMesh::create(std::vector<Polygon>{
        Polygon{{Vec2{0.0f, 0.0f}, Vec2{3.0e38f, 0.0f}, Vec2{0.0f, 3.0e38f}}}});
    if (!huge_mesh.has_value())
    {
        return 4;
    }
    if (!huge_mesh->contains(Vec2{1.0e38f, 1.0e38f}) || huge_mesh->contains(Vec2{2.5e38f, 1.5e38f}))
    {
        return 5;
    }
    const auto huge_path = find_path(*huge_mesh, Vec2{1.0e38f, 1.0e38f}, Vec2{0.5e38f, 0.4e38f});
    if (!huge_path.has_value() || huge_path->points.size() != 2u)
    {
        return 6;
    }

    // `length` and `normalized` stay finite and keep their direction for extreme input.
    constexpr float kHuge = std::numeric_limits<float>::max();
    const Vec2 unit = normalized(Vec2{-3.0e38f, 1.5e38f});
    const bool numerics_ok = std::isfinite(length(Vec2{kHuge, kHuge})) && std::isfinite(unit.x) &&
                             std::isfinite(unit.y) && unit.x < 0.0f && unit.y > 0.0f &&
                             std::isfinite(normalized(Vec2{0.0f, 0.0f}).x);
    if (!numerics_ok)
    {
        return 7;
    }

    const bool ok =
        (sim.agent(*id)->status == AgentStatus::Reached) && (sim.agent_count() == 1u) &&
        mesh->contains(Vec2{0.5f, 0.5f}) && !mesh->contains(Vec2{100.0f, 100.0f}) &&
        sim.set_goal(AgentId{99u}, Vec2{0.5f, 0.5f}).error().code == ErrorCode::NotFound;
    return ok ? 0 : 8;
}
