#include <vwmini/simulation.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

namespace {

using vwmini::AgentConfig;
using vwmini::AgentStatus;
using vwmini::ErrorCode;
using vwmini::NavMesh;
using vwmini::Polygon;
using vwmini::Vec2;

void require(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

[[nodiscard]] Polygon triangle(Vec2 a, Vec2 b, Vec2 c) {
    return {{{a, b, c}}};
}

[[nodiscard]] NavMesh square_mesh() {
    auto mesh =
        NavMesh::create({triangle({0, 0}, {4, 0}, {4, 4}), triangle({0, 0}, {4, 4}, {0, 4})});
    require(mesh.has_value(), "square mesh creates");
    return *mesh;
}

[[nodiscard]] NavMesh broad_mesh() {
    auto mesh = NavMesh::create({triangle({9990, -10}, {10020, -10}, {10020, 10}),
                                 triangle({9990, -10}, {10020, 10}, {9990, 10})});
    require(mesh.has_value(), "broad mesh creates");
    return *mesh;
}

[[nodiscard]] NavMesh corner_mesh() {
    auto triangles =
        vwmini::triangulate_simple_polygon({{{0, 0}, {10, 0}, {10, 2}, {2, 2}, {2, 10}, {0, 10}}});
    require(triangles.has_value(), "corner outline triangulates");
    auto mesh = NavMesh::create(*triangles);
    require(mesh.has_value(), "corner mesh creates");
    return *mesh;
}

void require_finite_and_bounded(const vwmini::AgentState &before, const vwmini::AgentState &after,
                                float seconds, const NavMesh &mesh, const char *message) {
    const double displacement_x = static_cast<double>(after.position.x) - before.position.x;
    const double displacement_y = static_cast<double>(after.position.y) - before.position.y;
    const double observed_speed = std::hypot(displacement_x, displacement_y) / seconds;
    const double reported_speed = std::hypot(after.velocity.x, after.velocity.y);
    require(std::isfinite(after.position.x) && std::isfinite(after.position.y) &&
                std::isfinite(after.velocity.x) && std::isfinite(after.velocity.y) &&
                observed_speed <= static_cast<double>(after.max_speed) + 1.0e-5 &&
                reported_speed <= static_cast<double>(after.max_speed) + 1.0e-5,
            message);
    require(mesh.contains(after.position), "stored movement remains contained");
}

void geometry_tests() {
    require(vwmini::normalized({0, 0}) == Vec2{}, "normalizing zero is zero");
    require(vwmini::length({3, 4}) == 5.0F, "vector length");

    auto clockwise = vwmini::triangulate_simple_polygon({{{0, 0}, {0, 1}, {1, 0}}});
    require(!clockwise && clockwise.error().code == ErrorCode::InvalidMesh, "clockwise rejected");
    auto bow_tie = vwmini::triangulate_simple_polygon({{{0, 0}, {2, 2}, {0, 2}, {2, 0}}});
    require(!bow_tie && bow_tie.error().code == ErrorCode::InvalidMesh,
            "self intersection rejected");
    auto collinear = vwmini::triangulate_simple_polygon({{{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}}});
    require(collinear && collinear->size() == 3, "collinear outline vertex triangulates");

    auto invalid =
        NavMesh::create({triangle({0, 0}, {1, 0}, {0, 1}), triangle({0, 0}, {1, 0}, {0.5F, 1})});
    require(!invalid && invalid.error().code == ErrorCode::InvalidMesh, "overlap rejected");
    auto non_finite =
        NavMesh::create({triangle({0, 0}, {1, 0}, {std::numeric_limits<float>::infinity(), 1})});
    require(!non_finite && non_finite.error().code == ErrorCode::InvalidArgument,
            "non-finite mesh vertex diagnosed");
    const NavMesh mesh = square_mesh();
    require(mesh.contains({2, 2}), "interior containment");
    require(mesh.contains({-0.00005F, 2}), "epsilon edge containment");
    require(!mesh.contains({-0.01F, 2}), "outside containment");
    require(!mesh.contains({std::numeric_limits<float>::quiet_NaN(), 0}), "non-finite containment");
}

void path_tests() {
    const NavMesh mesh = square_mesh();
    auto direct = vwmini::find_path(mesh, {0.5F, 0.5F}, {3.5F, 3.5F});
    require(direct && direct->points == std::vector<Vec2>{{0.5F, 0.5F}, {3.5F, 3.5F}},
            "direct path preserves endpoints");
    auto equal = vwmini::find_path(mesh, {1, 1}, {1, 1});
    require(equal && equal->points == std::vector<Vec2>{{1, 1}}, "equal path has one point");
    auto outside = vwmini::find_path(mesh, {-1, 1}, {1, 1});
    require(!outside && outside.error().code == ErrorCode::OutsideMesh,
            "outside endpoint diagnosed");

    auto l_triangles =
        vwmini::triangulate_simple_polygon({{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}});
    require(l_triangles.has_value(), "concave outline triangulates");
    auto l_mesh = NavMesh::create(*l_triangles);
    require(l_mesh.has_value(), "concave mesh creates");
    auto around_corner = vwmini::find_path(*l_mesh, {2.5F, 0.5F}, {0.5F, 2.5F});
    require(around_corner && around_corner->points.size() >= 3,
            "concave route is string-pulled around corner");

    auto disconnected =
        NavMesh::create({triangle({0, 0}, {1, 0}, {0, 1}), triangle({3, 0}, {4, 0}, {3, 1})});
    require(disconnected.has_value(), "disconnected mesh creates");
    auto no_path = vwmini::find_path(*disconnected, {0.1F, 0.1F}, {3.1F, 0.1F});
    require(!no_path && no_path.error().code == ErrorCode::NoPath, "disconnected path diagnosed");

    // These cells touch only at one vertex and must not form a route component.
    auto vertex_touch =
        NavMesh::create({triangle({0, 0}, {1, 0}, {0, 1}), triangle({1, 1}, {1, 0}, {2, 1})});
    require(vertex_touch.has_value(), "vertex-touch mesh creates");
    auto vertex_path = vwmini::find_path(*vertex_touch, {0.1F, 0.1F}, {1.5F, 0.5F});
    require(!vertex_path && vertex_path.error().code == ErrorCode::NoPath,
            "vertex touch is not adjacency");
}

void numeric_step_tests() {
    vwmini::Simulation denormal_simulation(square_mesh());
    auto denormal_agent = denormal_simulation.add_agent({{0, 0}, 0.1F, 1.0F, Vec2{3, 0}});
    require(denormal_agent.has_value(), "denormal agent adds");
    const auto before_denormal = *denormal_simulation.agent(*denormal_agent);
    const float denormal = std::numeric_limits<float>::denorm_min();
    require(denormal_simulation.step(denormal).has_value(), "denormal step succeeds");
    const auto after_denormal = *denormal_simulation.agent(*denormal_agent);
    require_finite_and_bounded(before_denormal, after_denormal, denormal, square_mesh(),
                               "denormal movement is finite and speed bounded");

    const NavMesh mesh = broad_mesh();
    vwmini::Simulation far_simulation(mesh);
    auto far_agent = far_simulation.add_agent({{10000, 0}, 0.1F, 1.0F, Vec2{10001, 0}});
    require(far_agent.has_value(), "far-origin agent adds");
    const auto before_far = *far_simulation.agent(*far_agent);
    constexpr float elapsed = 0.0006F;
    require(far_simulation.step(elapsed).has_value(), "far-origin small step succeeds");
    const auto after_far = *far_simulation.agent(*far_agent);
    require(after_far.position == before_far.position && after_far.velocity == Vec2{},
            "one-ULP overspeed becomes a no-op");
    require_finite_and_bounded(before_far, after_far, elapsed, mesh,
                               "far-origin stored movement is finite and bounded");

    vwmini::Simulation huge_duration_simulation(square_mesh());
    auto slow_agent =
        huge_duration_simulation.add_agent({{0.5F, 0.5F}, 0.1F, 0.001F, Vec2{3.5F, 0.5F}});
    require(slow_agent.has_value(), "huge-duration agent adds");
    require(huge_duration_simulation.step(std::numeric_limits<float>::max()).has_value(),
            "huge finite duration terminates");
    const auto slow_state = huge_duration_simulation.agent(*slow_agent);
    require(slow_state && slow_state->status == AgentStatus::Reached &&
                std::isfinite(slow_state->position.x) && std::isfinite(slow_state->velocity.x),
            "huge finite duration remains finite and reaches a reachable goal");
}

void simulation_tests() {
    vwmini::Simulation simulation(square_mesh());
    auto bad = simulation.add_agent({{0, 0}, -1.0F, 1.0F});
    require(!bad && bad.error().code == ErrorCode::InvalidArgument, "invalid agent diagnosed");
    auto agent = simulation.add_agent({{0.5F, 0.5F}, 0.2F, 1.0F, Vec2{3.5F, 0.5F}});
    require(agent.has_value() && agent->value != 0, "agent adds");
    require(simulation.agent(*agent)->status == AgentStatus::Moving, "goal starts moving");
    auto invalid_step = simulation.step(-1.0F);
    require(!invalid_step && invalid_step.error().code == ErrorCode::InvalidArgument,
            "invalid step diagnosed");
    require(simulation.step(4.0F).has_value(), "large step succeeds");
    const auto state = simulation.agent(*agent);
    require(state && state->status == AgentStatus::Reached && state->velocity == Vec2{},
            "agent reaches and stops");
    require(simulation.clear_goal(*agent).has_value() &&
                simulation.agent(*agent)->status == AgentStatus::Idle,
            "clear goal idles agent");
    require(simulation.remove_agent(*agent).has_value() && !simulation.agent(*agent),
            "removed agent disappears");
    const auto missing = simulation.clear_goal(*agent);
    require(!missing && missing.error().code == ErrorCode::NotFound, "removed id is not found");

    vwmini::Simulation crowd(square_mesh());
    auto first = crowd.add_agent({{0.5F, 1.5F}, 0.15F, 1.0F, Vec2{3.5F, 2.5F}});
    auto second = crowd.add_agent({{2.5F, 0.5F}, 0.15F, 1.0F, Vec2{1.5F, 3.5F}});
    require(first && second, "crossing crowd adds");
    for (int tick = 0; tick < 250; ++tick) {
        require(crowd.step(0.02F).has_value(), "crowd step succeeds");
        const auto a = crowd.agent(*first);
        const auto b = crowd.agent(*second);
        require(a && b &&
                    vwmini::length(a->position - b->position) >= a->radius + b->radius - 0.001F,
                "crossing discs do not overlap");
    }
    const auto first_state = crowd.agent(*first);
    const auto second_state = crowd.agent(*second);
    require(first_state->status == AgentStatus::Reached &&
                second_state->status == AgentStatus::Reached,
            "crossing agents make progress to their goals");

    vwmini::Simulation corner_crowd(corner_mesh());
    auto leading = corner_crowd.add_agent({{3.0F, 1.0F}, 0.25F, 1.0F, Vec2{1.0F, 8.0F}});
    auto trailing = corner_crowd.add_agent({{3.51F, 1.0F}, 0.25F, 1.0F, Vec2{1.0F, 7.0F}});
    require(leading && trailing, "corner-following crowd adds");
    for (int tick = 0; tick < 800; ++tick) {
        require(corner_crowd.step(0.02F).has_value(), "corner-following step succeeds");
        const auto lead = corner_crowd.agent(*leading);
        const auto trail = corner_crowd.agent(*trailing);
        require(lead && trail &&
                    std::hypot(static_cast<double>(lead->position.x) - trail->position.x,
                               static_cast<double>(lead->position.y) - trail->position.y) >=
                        lead->radius + trail->radius - 0.001,
                "corner-following stored endpoints remain separated");
    }
    const auto leading_state = corner_crowd.agent(*leading);
    const auto trailing_state = corner_crowd.agent(*trailing);
    require(leading_state->status == AgentStatus::Reached &&
                trailing_state->status == AgentStatus::Reached,
            "corner-following agents complete distinct goals");
}

} // namespace

int main() {
    geometry_tests();
    path_tests();
    numeric_step_tests();
    simulation_tests();
}
