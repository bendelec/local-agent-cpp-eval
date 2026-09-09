#include <vwmini/simulation.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Lifecycle, goal transitions, and single-agent stepping (SIM-005..009,
// SIM-012/013). Avoidance interactions live in simulation_avoidance_test.cpp.

namespace {

using namespace vwmini::test;
using vwmini::AgentConfig;
using vwmini::AgentId;
using vwmini::AgentState;
using vwmini::AgentStatus;
using vwmini::ErrorCode;
using vwmini::Simulation;
using vwmini::Vec2;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// -- Fixtures -----------------------------------------------------------------

/// 4x4 square at the origin, two CCW triangles.
vwmini::NavMesh make_square_mesh()
{
    return make_mesh(square_triangles(V(0, 0), 4.0f));
}

/// Two disjoint triangles: no route exists between them.
vwmini::NavMesh make_disconnected_mesh()
{
    return make_mesh({Tri(V(0, 0), V(1, 0), V(0, 1)), Tri(V(5, 5), V(6, 5), V(5, 6))});
}

// -- Operation helpers ----------------------------------------------------------
// add_or_fail / state_or_fail / make_l_mesh live in test_util.hpp.

void expect_ok(vwmini::Result<void> r)
{
    if (!r.has_value()) {
        ADD_FAILURE() << "unexpected error: " << r.error().message;
    }
}

template <class T> void expect_code(vwmini::Result<T> r, ErrorCode code)
{
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(code_of(r), code);
}

// ===== S4.1 Lifecycle (SIM-005/006/013) =======================================

TEST(SimulationLifecycle, DefaultConfigIsAcceptedAtMeshCorner)
{
    Simulation sim(make_square_mesh());
    const auto added = sim.add_agent(AgentConfig{});
    ASSERT_TRUE(added.has_value()) << added.error().message;
    const AgentState s = state_or_fail(sim, *added);
    EXPECT_EQ(s.position, V(0, 0));
    EXPECT_FLOAT_EQ(s.radius, 0.25f);
    EXPECT_FLOAT_EQ(s.max_speed, 1.4f);
    EXPECT_FALSE(s.goal.has_value());
    EXPECT_EQ(s.status, AgentStatus::Idle);
}

TEST(SimulationLifecycle, AddAgentReturnsLiveSnapshot)
{
    Simulation sim(make_square_mesh());
    const auto added = sim.add_agent(AgentConfig{V(1, 1), 0.3f, 2.0f, std::nullopt, -1.0f});
    ASSERT_TRUE(added.has_value()) << added.error().message;
    EXPECT_NE(added->value, 0u);
    EXPECT_EQ(sim.agent_count(), 1u);
    const AgentState s = state_or_fail(sim, *added);
    EXPECT_EQ(s.position, V(1, 1));
    EXPECT_EQ(s.velocity, V(0, 0));
    EXPECT_FLOAT_EQ(s.radius, 0.3f);
    EXPECT_FLOAT_EQ(s.max_speed, 2.0f);
    EXPECT_FALSE(s.goal.has_value());
    EXPECT_EQ(s.status, AgentStatus::Idle);
}

TEST(SimulationLifecycle, AddAgentRejectsNonFiniteValues)
{
    Simulation sim(make_square_mesh());
    struct Case {
        std::string name;
        AgentConfig config;
    };
    const std::vector<Case> cases = {
        {"position x NaN", {V(kNaN, 1), 0.3f, 1.0f, std::nullopt, -1.0f}},
        {"position y inf", {V(1, kInf), 0.3f, 1.0f, std::nullopt, -1.0f}},
        {"radius NaN", {V(1, 1), kNaN, 1.0f, std::nullopt, -1.0f}},
        {"radius inf", {V(1, 1), kInf, 1.0f, std::nullopt, -1.0f}},
        {"max_speed NaN", {V(1, 1), 0.3f, kNaN, std::nullopt, -1.0f}},
        {"goal NaN", {V(1, 1), 0.3f, 1.0f, V(kNaN, 2), -1.0f}},
        {"arrival NaN", {V(1, 1), 0.3f, 1.0f, std::nullopt, kNaN}},
    };
    for (const Case& c : cases) {
        const auto r = sim.add_agent(c.config);
        ASSERT_FALSE(r.has_value()) << c.name;
        EXPECT_EQ(code_of(r), ErrorCode::InvalidArgument) << c.name;
    }
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationLifecycle, AddAgentRejectsNonPositiveRadiusAndSpeed)
{
    Simulation sim(make_square_mesh());
    const std::vector<float> bad = {0.0f, -0.0f, -1.0f, -0.5f};
    for (const float v : bad) {
        EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(1, 1), v, 1.0f, std::nullopt, -1.0f})),
                  ErrorCode::InvalidArgument)
            << "radius " << v;
        EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(1, 1), 0.3f, v, std::nullopt, -1.0f})),
                  ErrorCode::InvalidArgument)
            << "max_speed " << v;
    }
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationLifecycle, ArrivalRadiusSentinelRules)
{
    // SIM-006: -1.0f is the sole negative sentinel; -0.0f is zero and valid.
    Simulation sim(make_square_mesh());
    const std::vector<float> valid = {-1.0f, -0.0f, 0.0f, 0.25f, 7.0f};
    for (const float a : valid) {
        EXPECT_TRUE(sim.add_agent(AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, a}).has_value())
            << "arrival " << a;
    }
    // Every other negative, on either side of -1, is invalid.
    const std::vector<float> invalid = {-0.5f, -0.999f, -1.001f, -2.0f, -100.0f};
    for (const float a : invalid) {
        EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, a})),
                  ErrorCode::InvalidArgument)
            << "arrival " << a;
    }
    EXPECT_EQ(sim.agent_count(), valid.size());
}

TEST(SimulationLifecycle, AddAgentRejectsOutsidePositionAndGoal)
{
    Simulation sim(make_square_mesh());
    EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(10, 10), 0.3f, 1.0f, std::nullopt, -1.0f})),
              ErrorCode::OutsideMesh);
    EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(1, 1), 0.3f, 1.0f, V(9, 9), -1.0f})),
              ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationLifecycle, ScalarValidationPrecedesContainment)
{
    Simulation sim(make_square_mesh());
    // A non-finite position is InvalidArgument, not OutsideMesh.
    EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(kNaN, 1), 0.3f, 1.0f, std::nullopt, -1.0f})),
              ErrorCode::InvalidArgument);
    // A non-positive radius wins over an outside position.
    EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(10, 10), 0.0f, 1.0f, std::nullopt, -1.0f})),
              ErrorCode::InvalidArgument);
    // A bad arrival sentinel wins over an outside goal.
    EXPECT_EQ(code_of(sim.add_agent(AgentConfig{V(1, 1), 0.3f, 1.0f, V(9, 9), -2.0f})),
              ErrorCode::InvalidArgument);
}

TEST(SimulationLifecycle, IdsAreUniqueAndNeverReused)
{
    Simulation sim(make_square_mesh());
    const AgentId a = add_or_fail(sim, AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, -1.0f});
    const AgentId b = add_or_fail(sim, AgentConfig{V(2, 1), 0.3f, 1.0f, std::nullopt, -1.0f});
    const AgentId c = add_or_fail(sim, AgentConfig{V(3, 1), 0.3f, 1.0f, std::nullopt, -1.0f});
    EXPECT_NE(a, b);
    EXPECT_NE(b, c);
    EXPECT_NE(a, c);
    expect_ok(sim.remove_agent(b));
    const AgentId d = add_or_fail(sim, AgentConfig{V(2, 2), 0.3f, 1.0f, std::nullopt, -1.0f});
    EXPECT_NE(d, a);
    EXPECT_NE(d, b); // Removed ids stay invalid (SIM-013).
    EXPECT_NE(d, c);
    EXPECT_EQ(sim.agent_count(), 3u);
    EXPECT_TRUE(sim.agent(a).has_value());
    EXPECT_FALSE(sim.agent(b).has_value());
    EXPECT_TRUE(sim.agent(d).has_value());
    EXPECT_EQ(state_or_fail(sim, d).position, V(2, 2));
}

TEST(SimulationLifecycle, RemoveAgentInvalidatesId)
{
    Simulation sim(make_square_mesh());
    const AgentId a = add_or_fail(sim, AgentConfig{V(1, 1), 0.3f, 1.0f, V(3, 3), -1.0f});
    expect_ok(sim.remove_agent(a));
    EXPECT_EQ(sim.agent_count(), 0u);
    EXPECT_FALSE(sim.agent(a).has_value());
    expect_code(sim.remove_agent(a), ErrorCode::NotFound);
    expect_code(sim.set_goal(a, V(2, 2)), ErrorCode::NotFound);
    expect_code(sim.clear_goal(a), ErrorCode::NotFound);
}

TEST(SimulationLifecycle, UnknownIdsReturnNotFound)
{
    Simulation sim(make_square_mesh());
    for (const AgentId id : {AgentId{0}, AgentId{7}, AgentId{0xFFFFFFFFu}}) {
        EXPECT_FALSE(sim.agent(id).has_value());
        expect_code(sim.remove_agent(id), ErrorCode::NotFound);
        expect_code(sim.set_goal(id, V(1, 1)), ErrorCode::NotFound);
        expect_code(sim.clear_goal(id), ErrorCode::NotFound);
    }
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationLifecycle, AgentSnapshotIsAnIndependentCopy)
{
    Simulation sim(make_square_mesh());
    const AgentId a = add_or_fail(sim, AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, -1.0f});
    AgentState copy = state_or_fail(sim, a);
    copy.position = V(99, 99);
    copy.velocity = V(5, 5);
    copy.status = AgentStatus::Reached;
    const AgentState fresh = state_or_fail(sim, a);
    EXPECT_EQ(fresh.position, V(1, 1));
    EXPECT_EQ(fresh.velocity, V(0, 0));
    EXPECT_EQ(fresh.status, AgentStatus::Idle);
}

TEST(SimulationLifecycle, MoveConstructorTransfersOwnership)
{
    Simulation source(make_square_mesh());
    const AgentId a = add_or_fail(source, AgentConfig{V(1, 1), 0.3f, 1.0f, V(3, 3), -1.0f});
    Simulation moved(std::move(source));

    EXPECT_EQ(moved.agent_count(), 1u);
    EXPECT_EQ(state_or_fail(moved, a).status, AgentStatus::Moving);
    expect_ok(moved.step(0.1f));
    EXPECT_NE(state_or_fail(moved, a).position, V(1, 1));

    // The moved-from simulation is inert but safe to query and to fail on.
    EXPECT_EQ(source.agent_count(), 0u);
    EXPECT_FALSE(source.agent(a).has_value());
    expect_code(source.add_agent(AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, -1.0f}),
                ErrorCode::InvalidArgument);
    expect_code(source.step(0.1f), ErrorCode::InvalidArgument);
    expect_code(source.remove_agent(a), ErrorCode::NotFound);
    expect_code(source.set_goal(a, V(2, 2)), ErrorCode::NotFound);
    expect_code(source.clear_goal(a), ErrorCode::NotFound);
}

TEST(SimulationLifecycle, MoveAssignmentTransfersOwnership)
{
    Simulation source(make_square_mesh());
    const AgentId a = add_or_fail(source, AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, -1.0f});
    Simulation target(make_square_mesh());
    target = std::move(source);

    EXPECT_EQ(target.agent_count(), 1u);
    EXPECT_TRUE(target.agent(a).has_value());
    EXPECT_EQ(source.agent_count(), 0u);
    expect_code(source.step(1.0f), ErrorCode::InvalidArgument);
}

// ===== S4.2 Goals and status transitions (SIM-005/007) =========================

TEST(SimulationGoals, SetGoalChecksLiveIdBeforeValidation)
{
    Simulation sim(make_square_mesh());
    expect_code(sim.set_goal(AgentId{9}, V(kNaN, kNaN), -5.0f), ErrorCode::NotFound);
}

TEST(SimulationGoals, SetGoalValidationLeavesStateUntouched)
{
    Simulation sim(make_square_mesh());
    const AgentId a = add_or_fail(sim, AgentConfig{V(1, 1), 0.3f, 1.0f, std::nullopt, -1.0f});
    expect_code(sim.set_goal(a, V(kNaN, 2)), ErrorCode::InvalidArgument);
    expect_code(sim.set_goal(a, V(kInf, 2)), ErrorCode::InvalidArgument);
    expect_code(sim.set_goal(a, V(2, 2), kNaN), ErrorCode::InvalidArgument);
    expect_code(sim.set_goal(a, V(2, 2), -2.0f), ErrorCode::InvalidArgument);
    expect_code(sim.set_goal(a, V(2, 2), -0.5f), ErrorCode::InvalidArgument);
    expect_code(sim.set_goal(a, V(10, 10)), ErrorCode::OutsideMesh);
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Idle);
    EXPECT_FALSE(s.goal.has_value());
    EXPECT_EQ(s.velocity, V(0, 0));
    EXPECT_EQ(s.position, V(1, 1));
}

TEST(SimulationGoals, SetGoalFarSetsMovingWithoutPresetVelocity)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 3.5f)));
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Moving);
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, V(3.5f, 3.5f));
    // Deliberate choice: Moving does not preset velocity; step derives it from
    // executed motion. The contract fixes zero velocity only for Reached,
    // NoPath, and Idle transitions.
    EXPECT_EQ(s.velocity, V(0, 0));
}

TEST(SimulationGoals, SetGoalWithinArrivalRadiusIsReachedImmediately)
{
    Simulation sim(make_square_mesh());
    const AgentId a = add_or_fail(sim, AgentConfig{V(1, 1), 0.25f, 1.0f, std::nullopt, -1.0f});

    // Sentinel: effective radius is the agent radius 0.25; distance 0.2 is within.
    expect_ok(sim.set_goal(a, V(1.2f, 1)));
    AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.velocity, V(0, 0));
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, V(1.2f, 1));

    // Explicit radius larger than the distance: Reached.
    expect_ok(sim.set_goal(a, V(1.3f, 1), 0.5f));
    EXPECT_EQ(state_or_fail(sim, a).status, AgentStatus::Reached);

    // Exact boundary: distance == arrival radius counts as within.
    expect_ok(sim.set_goal(a, V(1.5f, 1), 0.5f));
    EXPECT_EQ(state_or_fail(sim, a).status, AgentStatus::Reached);

    // Strictly farther than the explicit radius: Moving.
    expect_ok(sim.set_goal(a, V(1.6f, 1), 0.5f));
    EXPECT_EQ(state_or_fail(sim, a).status, AgentStatus::Moving);

    // Explicit zero arrival radius with a distinct goal: Moving.
    expect_ok(sim.set_goal(a, V(1.6f, 1), 0.0f));
    EXPECT_EQ(state_or_fail(sim, a).status, AgentStatus::Moving);
}

TEST(SimulationGoals, DisconnectedGoalSucceedsWithNoPathStatus)
{
    Simulation sim(make_disconnected_mesh());
    const AgentId a = add_or_fail(sim, AgentConfig{V(0.2f, 0.2f), 0.1f, 1.0f, std::nullopt, -1.0f});
    const auto r = sim.set_goal(a, V(5.2f, 5.2f));
    ASSERT_TRUE(r.has_value()) << r.error().message; // SIM-007: success, not an error.
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::NoPath);
    EXPECT_EQ(s.velocity, V(0, 0));
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, V(5.2f, 5.2f));
}

TEST(SimulationGoals, AddAgentWithGoalAppliesSetGoalTransitions)
{
    Simulation sim(make_square_mesh());
    const AgentId far =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, V(3.5f, 0.5f), -1.0f});
    EXPECT_EQ(state_or_fail(sim, far).status, AgentStatus::Moving);

    const AgentId near = add_or_fail(sim, AgentConfig{V(1, 1), 0.25f, 1.0f, V(1.1f, 1), -1.0f});
    EXPECT_EQ(state_or_fail(sim, near).status, AgentStatus::Reached);

    Simulation disconnected(make_disconnected_mesh());
    const AgentId split =
        add_or_fail(disconnected, AgentConfig{V(0.2f, 0.2f), 0.1f, 1.0f, V(5.2f, 5.2f), -1.0f});
    EXPECT_EQ(state_or_fail(disconnected, split).status, AgentStatus::NoPath);
}

TEST(SimulationGoals, SetGoalOverridesPreviousGoalAndRoute)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 0.5f)));
    expect_ok(sim.step(0.2f));
    const Vec2 toward_first = state_or_fail(sim, a).position;
    EXPECT_GT(toward_first.x, 0.5f);

    expect_ok(sim.set_goal(a, V(0.5f, 3.5f)));
    EXPECT_EQ(*state_or_fail(sim, a).goal, V(0.5f, 3.5f));
    expect_ok(sim.step(0.2f));
    const Vec2 toward_second = state_or_fail(sim, a).position;
    EXPECT_GT(toward_second.y, toward_first.y);
}

TEST(SimulationGoals, ClearGoalMakesAgentIdleAndImmobile)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 3.5f)));
    expect_ok(sim.step(0.1f));

    expect_ok(sim.clear_goal(a));
    const AgentState cleared = state_or_fail(sim, a);
    EXPECT_FALSE(cleared.goal.has_value());
    EXPECT_EQ(cleared.status, AgentStatus::Idle);
    EXPECT_EQ(cleared.velocity, V(0, 0));

    const Vec2 before = cleared.position;
    expect_ok(sim.step(1.0f));
    const AgentState after = state_or_fail(sim, a);
    EXPECT_EQ(after.position, before); // Later steps do not move it (SIM-007).
    EXPECT_EQ(after.velocity, V(0, 0));
    EXPECT_EQ(after.status, AgentStatus::Idle);

    expect_ok(sim.clear_goal(a)); // Idempotent on a live id.
    expect_code(sim.clear_goal(AgentId{42}), ErrorCode::NotFound);
}

TEST(SimulationGoals, ClearGoalFromReachedAndNoPathResetsToIdle)
{
    Simulation sim(make_square_mesh());
    const AgentId reached = add_or_fail(sim, AgentConfig{V(1, 1), 0.25f, 1.0f, V(1.1f, 1), -1.0f});
    ASSERT_EQ(state_or_fail(sim, reached).status, AgentStatus::Reached);
    expect_ok(sim.clear_goal(reached));
    EXPECT_EQ(state_or_fail(sim, reached).status, AgentStatus::Idle);
    EXPECT_FALSE(state_or_fail(sim, reached).goal.has_value());

    Simulation disconnected(make_disconnected_mesh());
    const AgentId split =
        add_or_fail(disconnected, AgentConfig{V(0.2f, 0.2f), 0.1f, 1.0f, V(5.2f, 5.2f), -1.0f});
    ASSERT_EQ(state_or_fail(disconnected, split).status, AgentStatus::NoPath);
    expect_ok(disconnected.clear_goal(split));
    EXPECT_EQ(state_or_fail(disconnected, split).status, AgentStatus::Idle);
}

// ===== S4.3 step validation and single-agent motion (SIM-008/009/012) ===========

TEST(SimulationStep, RejectsInvalidDurationsTransactionally)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 0.5f)));
    expect_ok(sim.step(0.1f));
    const AgentState before = state_or_fail(sim, a);

    for (const float dt : {kNaN, kInf, -kInf, -1.0f, -1e-30f}) {
        expect_code(sim.step(dt), ErrorCode::InvalidArgument);
        const AgentState after = state_or_fail(sim, a);
        EXPECT_EQ(after.position, before.position) << "dt " << dt;
        EXPECT_EQ(after.velocity, before.velocity) << "dt " << dt;
        EXPECT_EQ(after.status, before.status) << "dt " << dt;
        ASSERT_TRUE(after.goal.has_value());
        EXPECT_EQ(*after.goal, V(3.5f, 0.5f)) << "dt " << dt;
    }
}

TEST(SimulationStep, ZeroAndNegativeZeroDurationsAreNoOps)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 0.5f)));
    const AgentState before = state_or_fail(sim, a);

    expect_ok(sim.step(0.0f));
    expect_ok(sim.step(-0.0f)); // -0.0f is zero and valid (SIM-006 note).
    const AgentState after = state_or_fail(sim, a);
    EXPECT_EQ(after.position, before.position);
    EXPECT_EQ(after.velocity, before.velocity);
    EXPECT_EQ(after.status, before.status);
}

TEST(SimulationStep, StraightRunMovesAtMaxSpeedAlongRoute)
{
    const auto mesh = make_square_mesh();
    Simulation sim(mesh);
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 0.5f), 0.1f));

    expect_ok(sim.step(1.0f));
    const AgentState s = state_or_fail(sim, a);
    EXPECT_NEAR(s.position.x, 1.5f, 2e-4f);
    EXPECT_NEAR(s.position.y, 0.5f, 2e-4f);
    EXPECT_NEAR(s.velocity.x, 1.0f, 1e-4f);
    EXPECT_NEAR(s.velocity.y, 0.0f, 1e-4f);
    EXPECT_EQ(s.status, AgentStatus::Moving);
    EXPECT_TRUE(mesh.contains(s.position));
}

TEST(SimulationStep, DisplacementNeverExceedsMaxSpeedTimesDuration)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 0.8f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 3.5f)));
    const Vec2 before = state_or_fail(sim, a).position;
    expect_ok(sim.step(0.3f));
    const Vec2 after = state_or_fail(sim, a).position;
    const float travelled = length(after - before);
    EXPECT_LE(travelled, 0.8f * 0.3f + 1e-4f);
    EXPECT_GT(travelled, 0.2f); // Real progress toward the goal.
}

TEST(SimulationStep, TinyStepAtLargeCoordinatesNeverOvershootsBudget)
{
    // At |x| ~ 1e4 the float quantum (~9.8e-4) is larger than the whole
    // 1.4 * 0.0006 = 8.4e-4 metre budget of a tiny substep: storing the
    // double-precision landing as float used to round one ULP past the budget,
    // violating SIM-008's exact max_speed * dt cap. The step must now stay
    // within budget even when that means executing no motion at all.
    const vwmini::NavMesh mesh = make_mesh(square_triangles(V(9990.0f, -10.0f), 20.0f));
    Simulation sim(mesh);
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(10000.0f, 0.0f), 0.25f, 1.4f, V(10009.0f, 0.0f), 0.1f});
    const Vec2 before = state_or_fail(sim, a).position;
    expect_ok(sim.step(0.0006f));
    const AgentState after = state_or_fail(sim, a);
    const double moved = std::hypot(double(after.position.x) - double(before.x),
                                    double(after.position.y) - double(before.y));
    EXPECT_LE(moved, 1.4 * 0.0006); // Strict cap: no tolerance, this is SIM-008.
    EXPECT_TRUE(mesh.contains(after.position));
    EXPECT_EQ(after.status, AgentStatus::Moving);
    EXPECT_LE(length(after.velocity), 1.4f);
}

TEST(SimulationStep, LargeDurationStopsWithinArrivalRadiusWithoutOvershoot)
{
    const auto mesh = make_square_mesh();
    Simulation sim(mesh);
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 3.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 0.5f), 0.2f));

    expect_ok(sim.step(100.0f)); // Far more time than needed (SIM-008).
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.velocity, V(0, 0));
    // Stops inside the arrival band without passing the goal; exact landing on
    // the goal happens only with a zero arrival radius (tested separately).
    EXPECT_EQ(s.position.y, 0.5f);
    EXPECT_LE(s.position.x, 3.5f);
    EXPECT_GT(s.position.x, 3.3f);
    EXPECT_LE(std::abs(s.position.x - 3.5f), 0.2f);
    EXPECT_TRUE(mesh.contains(s.position));
}

TEST(SimulationStep, ZeroArrivalRadiusLandsExactlyOnGoal)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 0.5f), 0.0f));

    int steps = 0;
    while (state_or_fail(sim, a).status == AgentStatus::Moving && steps < 2000) {
        expect_ok(sim.step(1.0f / 60.0f));
        ++steps;
    }
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.position, V(3.5f, 0.5f));
    EXPECT_EQ(s.velocity, V(0, 0));
    EXPECT_LT(steps, 250); // ~3 m at 1 m/s with 1/60 s steps.
}

TEST(SimulationStep, ReachedIsStableTerminalState)
{
    Simulation sim(make_square_mesh());
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 3.0f, V(3.5f, 0.5f), 0.2f});
    expect_ok(sim.step(5.0f));
    const AgentState reached = state_or_fail(sim, a);
    ASSERT_EQ(reached.status, AgentStatus::Reached);

    expect_ok(sim.step(5.0f));
    const AgentState later = state_or_fail(sim, a);
    EXPECT_EQ(later.position, reached.position);
    EXPECT_EQ(later.velocity, V(0, 0));
    EXPECT_EQ(later.status, AgentStatus::Reached);
    ASSERT_TRUE(later.goal.has_value()); // The goal is retained (SIM-009/013).
    EXPECT_EQ(*later.goal, V(3.5f, 0.5f));
}

TEST(SimulationStep, IdleAndNoPathAgentsDoNotMove)
{
    Simulation square(make_square_mesh());
    const AgentId idle =
        add_or_fail(square, AgentConfig{V(1, 1), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(square.step(2.0f));
    EXPECT_EQ(state_or_fail(square, idle).position, V(1, 1));
    EXPECT_EQ(state_or_fail(square, idle).velocity, V(0, 0));
    EXPECT_EQ(state_or_fail(square, idle).status, AgentStatus::Idle);

    Simulation disconnected(make_disconnected_mesh());
    const AgentId split =
        add_or_fail(disconnected, AgentConfig{V(0.2f, 0.2f), 0.1f, 1.0f, V(5.2f, 5.2f), -1.0f});
    ASSERT_EQ(state_or_fail(disconnected, split).status, AgentStatus::NoPath);
    expect_ok(disconnected.step(2.0f));
    EXPECT_EQ(state_or_fail(disconnected, split).position, V(0.2f, 0.2f));
    EXPECT_EQ(state_or_fail(disconnected, split).velocity, V(0, 0));
    EXPECT_EQ(state_or_fail(disconnected, split).status, AgentStatus::NoPath);
}

TEST(SimulationStep, CornerRouteIsFollowedAndStaysContained)
{
    const auto mesh = make_l_mesh();
    Simulation sim(mesh);
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(1.5f, 0.5f), 0.15f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(0.5f, 2.5f), 0.05f));

    float min_corner_distance = 1e9f;
    int steps = 0;
    while (state_or_fail(sim, a).status == AgentStatus::Moving && steps < 3000) {
        expect_ok(sim.step(1.0f / 60.0f));
        ++steps;
        const AgentState s = state_or_fail(sim, a);
        ASSERT_TRUE(std::isfinite(s.position.x) && std::isfinite(s.position.y));
        ASSERT_TRUE(mesh.contains(s.position)) << "left the mesh at step " << steps;
        min_corner_distance = std::min(min_corner_distance, length(s.position - V(1.0f, 1.0f)));
    }
    const AgentState s = state_or_fail(sim, a);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_NEAR(s.position.x, 0.5f, 0.05f);
    EXPECT_NEAR(s.position.y, 2.5f, 0.05f);
    EXPECT_LT(steps, 400);
    // The route bends at the reflex corner: the walk passes through it.
    EXPECT_LT(min_corner_distance, 0.1f);
}

TEST(SimulationStep, HugeDurationStaysFiniteAndContained)
{
    const auto mesh = make_square_mesh();
    Simulation sim(mesh);
    const AgentId a =
        add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.0f, std::nullopt, -1.0f});
    expect_ok(sim.set_goal(a, V(3.5f, 3.5f)));
    expect_ok(sim.step(1e30f));
    const AgentState s = state_or_fail(sim, a);
    ASSERT_TRUE(std::isfinite(s.position.x) && std::isfinite(s.position.y));
    EXPECT_TRUE(mesh.contains(s.position));
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.position, V(3.5f, 3.5f));
}

TEST(SimulationStep, ReplayIsDeterministic)
{
    const auto run = [] {
        Simulation sim(make_square_mesh());
        const AgentId a =
            add_or_fail(sim, AgentConfig{V(0.5f, 0.5f), 0.25f, 1.2f, V(3.5f, 3.5f), -1.0f});
        const AgentId b =
            add_or_fail(sim, AgentConfig{V(3.5f, 0.5f), 0.2f, 0.9f, std::nullopt, -1.0f});
        std::vector<std::optional<AgentState>> trace;
        const auto record = [&](AgentId id) { trace.push_back(sim.agent(id)); };
        expect_ok(sim.step(0.1f));
        record(a);
        record(b);
        expect_ok(sim.step(1.0f / 60.0f));
        record(a);
        record(b);
        expect_ok(sim.set_goal(b, V(0.5f, 3.5f), 0.1f));
        record(a);
        record(b);
        expect_ok(sim.step(0.35f));
        record(a);
        record(b);
        expect_ok(sim.clear_goal(a));
        expect_ok(sim.step(2.0f));
        record(a);
        record(b);
        expect_ok(sim.remove_agent(b));
        expect_ok(sim.step(0.5f));
        record(a);
        record(b); // Removed: nullopt in both replays.
        return trace;
    };
    const auto first = run();
    const auto second = run();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        ASSERT_EQ(first[i].has_value(), second[i].has_value()) << "snapshot " << i;
        if (!first[i].has_value()) {
            continue;
        }
        EXPECT_EQ(first[i]->position, second[i]->position) << "snapshot " << i;
        EXPECT_EQ(first[i]->velocity, second[i]->velocity) << "snapshot " << i;
        EXPECT_EQ(first[i]->status, second[i]->status) << "snapshot " << i;
        EXPECT_EQ(first[i]->goal, second[i]->goal) << "snapshot " << i;
    }
}

} // namespace
