// Simulation lifecycle track (S): agent ids, configuration validation, goals, stepping,
// and arrival semantics (SIM-005..SIM-009, SIM-013).
//
// Uses a known-good fixed mesh created directly from literal triangles.

#include "conformance_fixture.hpp"

#include <vwmini/simulation.hpp>

#include <cmath>
#include <vector>

using namespace vwmini;
using namespace vwmini_conformance;

// ---- SIM-005: add_agent configuration validation ----

TEST(Simulation_AddAgent, NonFinitePositionIsInvalidArgument)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{std::nanf(""), 5.0f};
    expect_error(sim.add_agent(config), ErrorCode::InvalidArgument);
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(Simulation_AddAgent, NonPositiveRadiusOrSpeedIsInvalidArgument)
{
    Simulation sim(make_square_mesh());

    AgentConfig zero_radius;
    zero_radius.position = Vec2{5.0f, 5.0f};
    zero_radius.radius = 0.0f;
    expect_error(sim.add_agent(zero_radius), ErrorCode::InvalidArgument);

    AgentConfig negative_speed;
    negative_speed.position = Vec2{5.0f, 5.0f};
    negative_speed.max_speed = -1.0f;
    expect_error(sim.add_agent(negative_speed), ErrorCode::InvalidArgument);

    AgentConfig nan_speed;
    nan_speed.position = Vec2{5.0f, 5.0f};
    nan_speed.max_speed = std::nanf("");
    expect_error(sim.add_agent(nan_speed), ErrorCode::InvalidArgument);

    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(Simulation_AddAgent, PositionOutsideMeshIsOutsideMesh)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{-1.0f, 5.0f};
    expect_error(sim.add_agent(config), ErrorCode::OutsideMesh);
}

TEST(Simulation_AddAgent, GoalOutsideMeshIsOutsideMesh)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{5.0f, 11.0f};
    expect_error(sim.add_agent(config), ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(Simulation_AddAgent, NonFiniteGoalIsInvalidArgument)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{std::nanf(""), 5.0f};
    expect_error(sim.add_agent(config), ErrorCode::InvalidArgument);
}

TEST(Simulation_AddAgent, ValidAgentGetsNonZeroId)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    const auto result = sim.add_agent(config);
    ASSERT_TRUE(result.has_value());
    EXPECT_NE(result->value, 0u);
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(Simulation_AddAgent, OptionalGoalEstablishesRoute)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{2.0f, 5.0f};
    config.goal = Vec2{8.0f, 5.0f};
    config.max_speed = 1.0f;
    const auto result = sim.add_agent(config);
    ASSERT_TRUE(result.has_value());

    const auto state = sim.agent(*result);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Moving);
    EXPECT_TRUE(state->goal.has_value());
    EXPECT_EQ(*state->goal, (Vec2{8.0f, 5.0f}));
}

TEST(Simulation_AddAgent, DisconnectedGoalEstablishesNoPath)
{
    Simulation sim(make_disconnected_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 2.0f};
    config.goal = Vec2{25.0f, 2.0f};
    const auto result = sim.add_agent(config);
    ASSERT_TRUE(result.has_value());

    const auto state = sim.agent(*result);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::NoPath);
}

// ---- SIM-006: arrival-radius sentinel ----

TEST(Simulation_ArrivalRadius, NegativeOtherThanSentinelIsInvalidArgument)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.arrival_radius = -2.0f;
    expect_error(sim.add_agent(config), ErrorCode::InvalidArgument);

    config.arrival_radius = -0.5f;
    expect_error(sim.add_agent(config), ErrorCode::InvalidArgument);
}

TEST(Simulation_ArrivalRadius, SentinelUsesAgentRadius)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{5.2f, 5.0f}; // 0.2 away, within radius 0.25
    config.radius = 0.25f;
    config.arrival_radius = -1.0f;
    const auto result = sim.add_agent(config);
    ASSERT_TRUE(result.has_value());

    const auto state = sim.agent(*result);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
}

TEST(Simulation_ArrivalRadius, ZeroIsAnExplicitArrivalRadius)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{5.0f, 5.0f};
    config.arrival_radius = 0.0f;
    const auto result = sim.add_agent(config);
    ASSERT_TRUE(result.has_value());

    const auto state = sim.agent(*result);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
}

TEST(Simulation_ArrivalRadius, NegativeZeroIsAValidExplicitZero)
{
    // SIM-006 edge case: -0.0f is zero and valid.
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{5.0f, 5.0f};
    config.arrival_radius = -0.0f;
    const auto result = sim.add_agent(config);
    ASSERT_TRUE(result.has_value());
    const auto state = sim.agent(*result);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
}

// ---- SIM-007: goal and clear_goal state transitions ----

TEST(Simulation_SetGoal, UnknownIdIsNotFound)
{
    Simulation sim(make_square_mesh());
    expect_error(sim.set_goal(AgentId{42u}, Vec2{5.0f, 5.0f}), ErrorCode::NotFound);
}

TEST(Simulation_SetGoal, FarGoalSetsMoving)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{2.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    const auto result = sim.set_goal(id, Vec2{8.0f, 5.0f});
    ASSERT_TRUE(result.has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Moving);
    EXPECT_EQ(*state->goal, (Vec2{8.0f, 5.0f}));
}

TEST(Simulation_SetGoal, GoalWithinArrivalRadiusSetsReached)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    const auto result = sim.set_goal(id, Vec2{5.1f, 5.0f}, 0.25f);
    ASSERT_TRUE(result.has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
}

TEST(Simulation_SetGoal, DisconnectedGoalSetsNoPathWithZeroVelocity)
{
    Simulation sim(make_disconnected_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 2.0f};
    const auto id = sim.add_agent(config).value();

    const auto result = sim.set_goal(id, Vec2{25.0f, 2.0f});
    ASSERT_TRUE(result.has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::NoPath);
    EXPECT_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
}

TEST(Simulation_SetGoal, NegativeNonSentinelRadiusIsInvalidArgument)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    expect_error(sim.set_goal(id, Vec2{8.0f, 5.0f}, -2.0f), ErrorCode::InvalidArgument);
}

TEST(Simulation_ClearGoal, ClearsGoalAndSetsIdle)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{2.0f, 5.0f};
    config.goal = Vec2{8.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    const auto result = sim.clear_goal(id);
    ASSERT_TRUE(result.has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Idle);
    EXPECT_FALSE(state->goal.has_value());
    EXPECT_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
}

TEST(Simulation_ClearGoal, UnknownIdIsNotFound)
{
    Simulation sim(make_square_mesh());
    expect_error(sim.clear_goal(AgentId{7u}), ErrorCode::NotFound);
}

// ---- SIM-008 / SIM-009: step validation, motion bounds, arrival ----

TEST(Simulation_Step, NegativeOrNonFiniteDurationIsInvalidArgument)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{8.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    expect_error(sim.step(-1.0f), ErrorCode::InvalidArgument);
    expect_error(sim.step(std::nanf("")), ErrorCode::InvalidArgument);
    expect_error(sim.step(INFINITY), ErrorCode::InvalidArgument);

    // Transactional: no state changed.
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->position, (Vec2{5.0f, 5.0f}));
}

TEST(Simulation_Step, ZeroDurationChangesNothing)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.goal = Vec2{8.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    const auto result = sim.step(0.0f);
    ASSERT_TRUE(result.has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->position, (Vec2{5.0f, 5.0f}));
}

TEST(Simulation_Step, MotionStaysWithinMaxSpeedAndMesh)
{
    const NavMesh mesh = make_square_mesh();
    Simulation sim(mesh);
    AgentConfig config;
    config.position = Vec2{2.0f, 5.0f};
    config.goal = Vec2{8.0f, 5.0f};
    config.max_speed = 1.0f;
    const auto id = sim.add_agent(config).value();

    for (int i = 0; i < 30; ++i) {
        ASSERT_TRUE(sim.step(1.0f / 30.0f).has_value());
        const auto state = sim.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_TRUE(is_finite(state->position));
        EXPECT_TRUE(is_finite(state->velocity));
        EXPECT_TRUE(mesh.contains(state->position));
        EXPECT_LE(length(state->velocity), state->max_speed + kEps);
    }
}

TEST(Simulation_Step, AgentReachesGoalAndStops)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{1.0f, 5.0f};
    config.goal = Vec2{5.0f, 5.0f};
    config.max_speed = 2.0f;
    config.arrival_radius = 0.1f;
    const auto id = sim.add_agent(config).value();

    // Plenty of time to cover 4.0 metres at 2.0 m/s.
    for (int i = 0; i < 120; ++i) {
        ASSERT_TRUE(sim.step(1.0f / 30.0f).has_value());
    }
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
    EXPECT_LE(length(state->position - Vec2{5.0f, 5.0f}), 0.1f + kEps);
}

TEST(Simulation_Step, LargeDurationDoesNotOvershootGoal)
{
    // SIM-008: a large positive duration must not overshoot the goal.
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{1.0f, 5.0f};
    config.goal = Vec2{5.0f, 5.0f};
    config.max_speed = 2.0f;
    config.arrival_radius = 0.1f;
    const auto id = sim.add_agent(config).value();

    ASSERT_TRUE(sim.step(100.0f).has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_LE(length(state->position - Vec2{5.0f, 5.0f}), 0.1f + kEps);
}

TEST(Simulation_Step, BentRouteAroundReflexCornerReachesGoal)
{
    // A motion test, not only a find_path test: the route passes through the
    // reflex vertex of the L mesh. Advancing to the next waypoint before the
    // current corner is actually reached would make the next steering segment
    // cut through the missing quadrant and leave the agent clamped forever.
    const NavMesh mesh = make_l_mesh();
    Simulation sim(mesh);
    AgentConfig config;
    config.position = Vec2{1.9f, 0.55f};
    config.goal = Vec2{0.9f, 1.75f};
    config.radius = 0.05f;
    config.arrival_radius = 0.01f;
    config.max_speed = 1.4f;
    const auto id = sim.add_agent(config).value();

    // Deliberately use a normal public frame duration rather than a fixed
    // internal rate: clients such as the visual lab may legitimately call
    // step at 30 Hz. Correct waypoint transitions must not depend on the
    // caller's frame pacing.
    for (int step_index = 0; step_index < 120; ++step_index) {
        ASSERT_TRUE(sim.step(1.0f / 30.0f).has_value());
        const auto state = sim.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_TRUE(is_finite(state->position));
        EXPECT_TRUE(mesh.contains(state->position));
        EXPECT_LE(length(state->velocity), state->max_speed + kEps);
        if (state->status == AgentStatus::Reached) {
            break;
        }
    }

    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_LE(length(state->position - *config.goal), config.arrival_radius + kEps);
}

TEST(Simulation_Step, IdleAgentDoesNotMove)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    const auto id = sim.add_agent(config).value();

    ASSERT_TRUE(sim.step(1.0f).has_value());
    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::Idle);
    EXPECT_EQ(state->position, (Vec2{5.0f, 5.0f}));
    EXPECT_EQ(state->velocity, (Vec2{0.0f, 0.0f}));
}

// ---- SIM-013: queries and removal ----

TEST(Simulation_Query, AgentSnapshotAndCountAreExact)
{
    Simulation sim(make_square_mesh());
    EXPECT_EQ(sim.agent_count(), 0u);

    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    config.radius = 0.3f;
    config.max_speed = 1.2f;
    const auto id = sim.add_agent(config).value();
    EXPECT_EQ(sim.agent_count(), 1u);

    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->position, (Vec2{5.0f, 5.0f}));
    EXPECT_EQ(state->radius, 0.3f);
    EXPECT_EQ(state->max_speed, 1.2f);
    EXPECT_EQ(state->status, AgentStatus::Idle);
}

TEST(Simulation_Query, UnknownIdReturnsEmpty)
{
    Simulation sim(make_square_mesh());
    EXPECT_FALSE(sim.agent(AgentId{1u}).has_value());
}

TEST(Simulation_RemoveAgent, InvalidatesId)
{
    Simulation sim(make_square_mesh());
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    const auto id = sim.add_agent(config).value();
    EXPECT_EQ(sim.agent_count(), 1u);

    ASSERT_TRUE(sim.remove_agent(id).has_value());
    EXPECT_EQ(sim.agent_count(), 0u);
    EXPECT_FALSE(sim.agent(id).has_value());

    // All later id-based operations report NotFound.
    expect_error(sim.remove_agent(id), ErrorCode::NotFound);
    expect_error(sim.set_goal(id, Vec2{5.0f, 5.0f}), ErrorCode::NotFound);
    expect_error(sim.clear_goal(id), ErrorCode::NotFound);
}
