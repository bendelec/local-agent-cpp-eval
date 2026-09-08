#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <optional>
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
    const bool ok =
        (sim.agent(*id)->status == AgentStatus::Reached) && (sim.agent_count() == 1u) &&
        mesh->contains(Vec2{0.5f, 0.5f}) && !mesh->contains(Vec2{100.0f, 100.0f}) &&
        sim.set_goal(AgentId{99u}, Vec2{0.5f, 0.5f}).error().code == ErrorCode::NotFound;
    return ok ? 0 : 4;
}
