#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <utility>
#include <vector>

// End-to-end pipeline tests (S5.1): triangulate -> NavMesh::create -> find_path
// -> Simulation on one fixed L-shaped outline, plus deterministic replay and
// error propagation through the full stack. Requirement IDs are cited per test.

namespace {

using namespace vwmini::test;
using vwmini::AgentConfig;
using vwmini::AgentId;
using vwmini::AgentState;
using vwmini::AgentStatus;
using vwmini::ErrorCode;
using vwmini::NavMesh;
using vwmini::Polygon;
using vwmini::Simulation;
using vwmini::Vec2;
using vwmini::find_path;
using vwmini::length;
using vwmini::triangulate_simple_polygon;

// -- Fixed pipeline inputs ----------------------------------------------------
// l_outline / make_l_mesh / add_or_fail / state_or_fail live in test_util.hpp.

constexpr Vec2 kStart{1.75f, 0.5f}; // Strictly inside the horizontal bar.
constexpr Vec2 kGoal{0.5f, 2.5f};   // Strictly inside the vertical arm.
constexpr float kDt = 1.0f / 60.0f;
constexpr int kStepBudget = 2000;

/// Agent used by the simulation tests: radius 0.1, max_speed 1.0, goal from the
/// start, default arrival-radius sentinel (-1 resolves to the agent radius).
AgentConfig l_agent_config()
{
    return AgentConfig{kStart, 0.1f, 1.0f, kGoal, -1.0f};
}

// -- Full pipeline: outline -> triangles -> mesh -> path -----------------------

// Demonstrates MSH-002/003/004 (valid outline -> deterministic non-degenerate
// triangles -> accepted immutable mesh with the accepted triangle count,
// MSH-007), MSH-006 (one containment policy), and SIM-001/002/003 (contained,
// endpoint-preserving route that bends around the reflex corner).
TEST(Integration, FullPipelineOnLMesh)
{
    // Given: the fixed L outline.
    const Polygon outline = Poly(l_outline());

    // When: triangulate, then create the mesh.
    auto triangles = triangulate_simple_polygon(outline);
    ASSERT_TRUE(triangles.has_value()) << triangles.error().message;
    EXPECT_GE(triangles->size(), 2u); // The concave L needs at least two ears.
    const std::size_t triangle_count = triangles->size();
    auto mesh = NavMesh::create(std::move(*triangles));
    ASSERT_TRUE(mesh.has_value()) << mesh.error().message;
    EXPECT_EQ(mesh->cell_count(), triangle_count);

    // Then: containment sanity in both arms and outside the notch.
    EXPECT_TRUE(mesh->contains(kStart));          // bar
    EXPECT_TRUE(mesh->contains(V(1.5f, 0.25f)));  // bar
    EXPECT_TRUE(mesh->contains(kGoal));           // arm
    EXPECT_TRUE(mesh->contains(V(0.25f, 1.5f)));  // arm
    EXPECT_FALSE(mesh->contains(V(1.5f, 1.5f)));  // notch
    EXPECT_FALSE(mesh->contains(V(1.5f, 2.5f)));  // notch
    EXPECT_FALSE(mesh->contains(V(-0.5f, 0.5f))); // left of the bar
    EXPECT_FALSE(mesh->contains(V(0.5f, 3.5f)));  // above the arm

    // When: query a route from the bar into the arm.
    auto path = find_path(*mesh, kStart, kGoal);

    // Then: a contained polyline preserving both endpoints. The straight
    // start->goal segment crosses the notch, so the route must bend: at least
    // one intermediate point around the reflex corner (SIM-003).
    ASSERT_TRUE(path.has_value()) << path.error().message;
    ASSERT_GE(path->points.size(), 3u);
    EXPECT_EQ(path->points.front(), kStart);
    EXPECT_EQ(path->points.back(), kGoal);
    bool in_bar = false;
    bool in_arm = false;
    for (const Vec2 p : path->points) {
        EXPECT_TRUE(mesh->contains(p)); // SIM-002: every point is contained.
        in_bar = in_bar || (p.x > 0.0f && p.x < 2.0f && p.y > 0.0f && p.y < 1.0f);
        in_arm = in_arm || (p.x > 0.0f && p.x < 1.0f && p.y > 1.0f && p.y < 3.0f);
    }
    EXPECT_TRUE(in_bar); // The route touches the horizontal bar...
    EXPECT_TRUE(in_arm); // ...and the vertical arm: it turns the corner.
}

// -- Full pipeline: the simulation walks the routed corner ----------------------

// Demonstrates SIM-005 (a valid goal in AgentConfig establishes the initial
// Moving route), SIM-008 (every step succeeds; the agent never exceeds
// max_speed and never leaves the mesh), SIM-006 (the -1 sentinel resolves the
// arrival radius to the agent radius), and SIM-009 (Reached is a stable,
// zero-velocity terminal state).
TEST(Integration, SimulationFollowsPathOnLMesh)
{
    // Given: the pipeline-built mesh and one agent with its goal.
    const NavMesh mesh = make_l_mesh();
    Simulation sim(mesh);
    const AgentId a = add_or_fail(sim, l_agent_config());
    ASSERT_EQ(state_or_fail(sim, a).status, AgentStatus::Moving);

    // When: stepping at a fixed 1/60 s cadence until the goal is reached.
    AgentStatus status = AgentStatus::Moving;
    int steps = 0;
    while (status == AgentStatus::Moving && steps < kStepBudget) {
        const auto stepped = sim.step(kDt);
        ASSERT_TRUE(stepped.has_value()) << stepped.error().message;
        ++steps;
        const AgentState s = state_or_fail(sim, a);
        ASSERT_NE(s.status, AgentStatus::NoPath); // The L is one component.
        ASSERT_TRUE(mesh.contains(s.position)) << "left the mesh at step " << steps;
        ASSERT_LE(length(s.velocity), 1.0f + 1e-4f) << "too fast at step " << steps;
        status = s.status;
    }

    // Then: arrived within budget, inside the effective arrival radius
    // (the default sentinel resolves to the agent radius 0.1), at rest.
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_LT(steps, kStepBudget);
    EXPECT_LE(length(kGoal - s.position), 0.1f + 1e-4f);
    EXPECT_EQ(s.velocity, V(0, 0));
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, kGoal); // The goal is retained after arrival.

    // And: Reached is stable — one further step changes nothing (SIM-009).
    const auto settled = sim.step(kDt);
    ASSERT_TRUE(settled.has_value()) << settled.error().message;
    const AgentState after = state_or_fail(sim, a);
    EXPECT_EQ(after.status, AgentStatus::Reached);
    EXPECT_EQ(after.position, s.position);
    EXPECT_EQ(after.velocity, V(0, 0));
    ASSERT_TRUE(after.goal.has_value());
    EXPECT_EQ(*after.goal, kGoal);
}

// -- Deterministic replay -------------------------------------------------------

/// Everything one pipeline run observes: the route points and the full agent
/// state sequence from just after add_agent through the Reached step.
struct Replay {
    std::vector<Vec2> path_points;
    std::vector<AgentState> states;
};

/// Runs triangulate -> create -> find_path -> simulate-to-Reached once, on
/// fresh objects and with fixed literal inputs only.
Replay run_pipeline()
{
    Replay replay;
    const NavMesh mesh = make_l_mesh();

    auto path = find_path(mesh, kStart, kGoal);
    if (!path.has_value()) {
        ADD_FAILURE() << "find_path failed: " << path.error().message;
        return replay;
    }
    replay.path_points = path->points;

    Simulation sim(mesh);
    const AgentId a = add_or_fail(sim, l_agent_config());
    replay.states.push_back(state_or_fail(sim, a));
    int steps = 0;
    while (replay.states.back().status == AgentStatus::Moving && steps < kStepBudget) {
        const auto stepped = sim.step(kDt);
        if (!stepped.has_value()) {
            ADD_FAILURE() << "step failed: " << stepped.error().message;
            return replay;
        }
        ++steps;
        replay.states.push_back(state_or_fail(sim, a));
    }
    return replay;
}

// Demonstrates MSH-003 (identical input -> identical ordered triangles),
// SIM-002/SIM-004 (identical queries -> exactly equal point sequences and the
// same selected route), SIM-012 (the simulation is deterministic), and
// NFR-007 (tests are reproducible). Comparisons are exact: Vec2::operator==
// is component-wise equality, so equal runs are bitwise identical.
TEST(Integration, DeterministicReplay)
{
    // Given/When: the same pipeline twice, on fresh objects.
    const Replay first = run_pipeline();
    const Replay second = run_pipeline();

    // Then: identical routes, exactly.
    ASSERT_FALSE(first.path_points.empty());
    ASSERT_EQ(first.path_points.size(), second.path_points.size());
    for (std::size_t i = 0; i < first.path_points.size(); ++i) {
        EXPECT_EQ(first.path_points[i], second.path_points[i]) << "point " << i;
    }

    // Then: identical state sequences, exactly, ending in Reached.
    ASSERT_FALSE(first.states.empty());
    ASSERT_EQ(first.states.size(), second.states.size());
    for (std::size_t i = 0; i < first.states.size(); ++i) {
        const AgentState& f = first.states[i];
        const AgentState& s = second.states[i];
        EXPECT_EQ(f.position, s.position) << "state " << i;
        EXPECT_EQ(f.velocity, s.velocity) << "state " << i;
        EXPECT_EQ(f.radius, s.radius) << "state " << i;
        EXPECT_EQ(f.max_speed, s.max_speed) << "state " << i;
        EXPECT_EQ(f.goal, s.goal) << "state " << i;
        EXPECT_EQ(f.status, s.status) << "state " << i;
    }
    EXPECT_EQ(first.states.back().status, AgentStatus::Reached);
    EXPECT_EQ(second.states.back().status, AgentStatus::Reached);
}

// -- Error semantics survive the full stack --------------------------------------

// Demonstrates SIM-001 (an out-of-mesh start fails find_path with
// OutsideMesh), SIM-005 (an out-of-mesh position fails add_agent with
// OutsideMesh and creates no agent), SIM-007 (a failed set_goal is
// transactional: goal, status, and motion state are preserved), and NFR-006
// (predictable failure reporting). The codes are unit-tested elsewhere; this
// only proves they propagate through the composed pipeline.
TEST(Integration, ErrorPropagationThroughPipeline)
{
    const NavMesh mesh = make_l_mesh();

    // SIM-001: a start outside the mesh fails the query.
    EXPECT_EQ(code_of(find_path(mesh, V(5, 5), kGoal)), ErrorCode::OutsideMesh);

    Simulation sim(mesh);

    // SIM-005: an outside position is rejected; nothing is created.
    EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(5, 5), 0.1f, 1.0f, kGoal, -1.0f})),
              ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 0u);

    // SIM-007: a failed set_goal leaves the live agent exactly as it was.
    const AgentId a = add_or_fail(sim, l_agent_config());
    const AgentState before = state_or_fail(sim, a);
    ASSERT_EQ(before.status, AgentStatus::Moving);
    EXPECT_EQ(code_of(sim.set_goal(a, V(5, 5))), ErrorCode::OutsideMesh);
    const AgentState after = state_or_fail(sim, a);
    EXPECT_EQ(after.status, before.status);
    EXPECT_EQ(after.position, before.position);
    EXPECT_EQ(after.velocity, before.velocity);
    ASSERT_TRUE(after.goal.has_value());
    EXPECT_EQ(*after.goal, kGoal); // The previous goal survives the failure.
    EXPECT_EQ(sim.agent_count(), 1u);
}

} // namespace
