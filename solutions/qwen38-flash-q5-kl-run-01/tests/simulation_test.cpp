#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include "mesh_fixtures.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace
{

using vwmini::AgentConfig;
using vwmini::AgentId;
using vwmini::AgentState;
using vwmini::AgentStatus;
using vwmini::ErrorCode;
using vwmini::NavMesh;
using vwmini::Simulation;
using vwmini::Vec2;

/// An agent in the middle of a 10 m square of walkable space.
[[nodiscard]] AgentConfig centered_config()
{
    AgentConfig config;
    config.position = Vec2{5.0f, 5.0f};
    return config;
}

void expect_states_equal(const std::optional<AgentState> &left,
                         const std::optional<AgentState> &right)
{
    ASSERT_TRUE(left.has_value() == right.has_value());
    if (!left.has_value())
    {
        return;
    }
    EXPECT_EQ(left->position, right->position);
    EXPECT_EQ(left->velocity, right->velocity);
    EXPECT_EQ(left->radius, right->radius);
    EXPECT_EQ(left->max_speed, right->max_speed);
    EXPECT_EQ(left->goal, right->goal);
    EXPECT_EQ(left->status, right->status);
}

[[nodiscard]] float dist(Vec2 a, Vec2 b) noexcept
{
    return vwmini::length(a - b);
}

/// SIM-011: two discs may not interpenetrate by more than the acceptance tolerance.
void expect_no_overlap(const Simulation &sim, AgentId first, AgentId second)
{
    const auto a = sim.agent(first);
    const auto b = sim.agent(second);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_GE(dist(a->position, b->position), a->radius + b->radius - 1.0e-3f)
        << "agents " << a->position.x << " and " << b->position.x << " overlap";
}

/// SIM-008 / SIM-012: every agent stays finite, inside the mesh, and inside its speed limit.
void expect_physical_invariants(const Simulation &sim, const NavMesh &mesh,
                                const std::vector<AgentId> &ids)
{
    for (const AgentId id : ids)
    {
        const auto state = sim.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_TRUE(std::isfinite(state->position.x) && std::isfinite(state->position.y));
        EXPECT_TRUE(mesh.contains(state->position))
            << "agent left the mesh at (" << state->position.x << "," << state->position.y << ")";
        EXPECT_LE(vwmini::length(state->velocity), state->max_speed + 1.0e-4f)
            << "agent exceeded its maximum speed";
    }
}

} // namespace

TEST(SimulationTest, RejectsNonFiniteAgentConfiguration)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    AgentConfig config = centered_config();
    config.position = Vec2{nan, 1.0f};
    EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument);
    config = centered_config();
    config.position = Vec2{1.0f, infinity};
    EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument);
    config = centered_config();
    config.radius = nan;
    EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument);
    config = centered_config();
    config.max_speed = nan;
    EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument);
    config = centered_config();
    config.goal = Vec2{nan, 5.0f};
    EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument);
    config = centered_config();
    config.arrival_radius = nan;
    EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationTest, RejectsNonPositiveRadiusAndSpeed)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    for (const float bad : {0.0f, -0.5f})
    {
        AgentConfig radius = centered_config();
        radius.radius = bad;
        EXPECT_EQ(sim.add_agent(radius).error().code, ErrorCode::InvalidArgument);
        AgentConfig speed = centered_config();
        speed.max_speed = bad;
        EXPECT_EQ(sim.add_agent(speed).error().code, ErrorCode::InvalidArgument);
    }
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationTest, ArrivalRadiusHasExactlyOneNegativeSentinel)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    for (const float invalid : {-2.0f, -1.5f, -0.5f, -1.0e-8f})
    {
        AgentConfig config = centered_config();
        config.arrival_radius = invalid;
        EXPECT_EQ(sim.add_agent(config).error().code, ErrorCode::InvalidArgument)
            << "arrival radius " << invalid;
    }
    EXPECT_EQ(sim.agent_count(), 0u);

    // -1 means "use the agent radius": 0.4 m away with a 0.5 m radius is already arrived.
    AgentConfig sentinel = centered_config();
    sentinel.radius = 0.5f;
    sentinel.goal = Vec2{5.4f, 5.0f};
    const auto sentinel_id = sim.add_agent(sentinel);
    ASSERT_TRUE(sentinel_id.has_value());
    EXPECT_EQ(sim.agent(*sentinel_id)->status, AgentStatus::Reached);

    // -0.0f is zero, not the sentinel, and an explicit zero radius still accepts the
    // goal because the agent already sits exactly on it.
    AgentConfig zero = centered_config();
    zero.arrival_radius = -0.0f;
    zero.goal = Vec2{5.0f, 5.0f};
    const auto zero_id = sim.add_agent(zero);
    ASSERT_TRUE(zero_id.has_value());
    EXPECT_EQ(sim.agent(*zero_id)->status, AgentStatus::Reached);

    // set_goal applies the same sentinel rule.
    const auto id = sim.add_agent(centered_config()).value();
    EXPECT_EQ(sim.set_goal(id, Vec2(6.0f, 5.0f), -3.0f).error().code, ErrorCode::InvalidArgument);
    EXPECT_EQ(sim.set_goal(id, Vec2(6.0f, 5.0f), -0.5f).error().code, ErrorCode::InvalidArgument);
    EXPECT_TRUE(sim.set_goal(id, Vec2{6.0f, 5.0f}, 0.0f).has_value());
    EXPECT_EQ(sim.agent(id)->status, AgentStatus::Moving);
}

TEST(SimulationTest, ReportsPointsOutsideTheMesh)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(2.0f)));
    AgentConfig outside;
    outside.position = Vec2{3.0f, 1.0f};
    EXPECT_EQ(sim.add_agent(outside).error().code, ErrorCode::OutsideMesh);

    AgentConfig goal_outside;
    goal_outside.position = Vec2{1.0f, 1.0f};
    goal_outside.goal = Vec2{3.0f, 1.0f};
    EXPECT_EQ(sim.add_agent(goal_outside).error().code, ErrorCode::OutsideMesh);

    const auto id =
        sim.add_agent(AgentConfig{Vec2{1.0f, 1.0f}, 0.25f, 1.0f, std::nullopt, -1.0f}).value();
    EXPECT_EQ(sim.set_goal(id, Vec2(5.0f, 1.0f)).error().code, ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(SimulationTest, AddAgentWithGoalUsesTheGoalTransitions)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));

    AgentConfig far = centered_config();
    far.goal = Vec2{8.0f, 5.0f};
    const auto far_id = sim.add_agent(far);
    ASSERT_TRUE(far_id.has_value());
    EXPECT_EQ(sim.agent(*far_id)->status, AgentStatus::Moving);

    AgentConfig nearby = centered_config();
    nearby.radius = 0.25f;
    nearby.goal = Vec2{5.1f, 5.0f};
    const auto nearby_id = sim.add_agent(nearby);
    ASSERT_TRUE(nearby_id.has_value());
    EXPECT_EQ(sim.agent(*nearby_id)->status, AgentStatus::Reached);
    EXPECT_EQ(sim.agent(*nearby_id)->velocity, Vec2(0.0f, 0.0f));
}

TEST(SimulationTest, DisconnectedInMeshGoalSetsNoPathWithZeroVelocity)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::two_islands()));
    const auto id =
        sim.add_agent(AgentConfig{Vec2{0.25f, 0.25f}, 0.25f, 1.0f, std::nullopt, -1.0f}).value();
    EXPECT_TRUE(sim.set_goal(id, Vec2{20.25f, 0.25f}).has_value());

    const auto state = sim.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->status, AgentStatus::NoPath);
    EXPECT_EQ(state->velocity, Vec2(0.0f, 0.0f));
    EXPECT_EQ(state->goal, Vec2(20.25f, 0.25f));
}

TEST(SimulationTest, UnknownAndRemovedIdsReturnNotFound)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const AgentId unknown{42u};
    EXPECT_EQ(sim.set_goal(unknown, Vec2(5.0f, 5.0f)).error().code, ErrorCode::NotFound);
    EXPECT_EQ(sim.clear_goal(unknown).error().code, ErrorCode::NotFound);
    EXPECT_EQ(sim.remove_agent(unknown).error().code, ErrorCode::NotFound);
    EXPECT_FALSE(sim.agent(unknown).has_value());

    const auto added = sim.add_agent(centered_config());
    ASSERT_TRUE(added.has_value());
    const AgentId id = *added;
    EXPECT_NE(id.value, 0u);
    ASSERT_TRUE(sim.remove_agent(id).has_value());
    EXPECT_EQ(sim.agent_count(), 0u);
    EXPECT_FALSE(sim.agent(id).has_value());
    EXPECT_EQ(sim.set_goal(id, Vec2(6.0f, 5.0f)).error().code, ErrorCode::NotFound);
    EXPECT_EQ(sim.clear_goal(id).error().code, ErrorCode::NotFound);
    EXPECT_EQ(sim.remove_agent(id).error().code, ErrorCode::NotFound);

    // A removed id is never handed out again.
    const auto next = sim.add_agent(centered_config());
    ASSERT_TRUE(next.has_value());
    EXPECT_NE(*next, id);
}

TEST(SimulationTest, RejectedGoalLeavesThePreviousGoalInPlace)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto id =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.2f, 1.0f, Vec2{8.0f, 5.0f}, -1.0f}).value();
    const auto before = sim.agent(id);

    EXPECT_EQ(sim.set_goal(id, Vec2(200.0f, 5.0f)).error().code, ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.set_goal(id, Vec2(std::numeric_limits<float>::quiet_NaN(), 5.0f)).error().code,
              ErrorCode::InvalidArgument);
    expect_states_equal(sim.agent(id), before);
}

TEST(SimulationTest, ClearGoalMakesTheAgentIdleAndStopsIt)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto id =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.2f, 2.0f, Vec2{8.0f, 5.0f}, -1.0f}).value();
    ASSERT_TRUE(sim.step(0.5f).has_value());
    EXPECT_EQ(sim.agent(id)->status, AgentStatus::Moving);

    ASSERT_TRUE(sim.clear_goal(id).has_value());
    const auto stopped = sim.agent(id);
    EXPECT_EQ(stopped->status, AgentStatus::Idle);
    EXPECT_EQ(stopped->velocity, Vec2(0.0f, 0.0f));
    EXPECT_FALSE(stopped->goal.has_value());

    ASSERT_TRUE(sim.step(2.0f).has_value());
    EXPECT_EQ(sim.agent(id)->position, stopped->position);
}

TEST(SimulationTest, StepValidationIsTransactional)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto id =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.25f, 1.0f, Vec2{8.0f, 5.0f}, -1.0f}).value();
    const auto before = sim.agent(id);

    for (const float bad :
         {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -0.5f})
    {
        EXPECT_EQ(sim.step(bad).error().code, ErrorCode::InvalidArgument) << "duration " << bad;
        expect_states_equal(sim.agent(id), before);
    }
    ASSERT_TRUE(sim.step(0.0f).has_value());
    expect_states_equal(sim.agent(id), before);
}

TEST(SimulationTest, AgentMovesToGoalAndReachesIt)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair(10.0f));
    auto sim = Simulation(NavMesh(mesh));
    const auto id =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.2f, 2.0f, Vec2{8.0f, 5.0f}, 0.3f}).value();

    Vec2 previous = sim.agent(id)->position;
    float travelled = 0.0f;
    for (int index = 0; sim.agent(id)->status == AgentStatus::Moving; ++index)
    {
        ASSERT_LT(index, 400) << "agent never arrived";
        ASSERT_TRUE(sim.step(0.05f).has_value());
        const auto state = sim.agent(id);
        EXPECT_LE(vwmini::length(state->velocity), 2.0f + 1.0e-4f);
        EXPECT_TRUE(mesh.contains(state->position))
            << "(" << state->position.x << "," << state->position.y << ") left the mesh";
        travelled += dist(state->position, previous);
        previous = state->position;
    }

    const auto state = sim.agent(id);
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_EQ(state->velocity, Vec2(0.0f, 0.0f));
    EXPECT_LE(dist(state->position, *state->goal), 0.3f + 1.0e-4f);
    // Motion is incremental rather than a jump to the goal.
    EXPECT_GT(travelled, 3.0f);
    EXPECT_GT(travelled, 1.0e-3f);
}

TEST(SimulationTest, LongDurationDoesNotOvershootTheGoal)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto id =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.2f, 2.0f, Vec2{8.0f, 5.0f}, 0.3f}).value();
    ASSERT_TRUE(sim.step(500.0f).has_value());

    const auto state = sim.agent(id);
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_LE(dist(state->position, *state->goal), 0.3f + 1.0e-3f);
    EXPECT_EQ(state->velocity, Vec2(0.0f, 0.0f));
}

TEST(SimulationTest, AgentWithoutGoalStaysPut)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto id = sim.add_agent(centered_config()).value();
    ASSERT_TRUE(sim.step(3.0f).has_value());

    const auto state = sim.agent(id);
    EXPECT_EQ(state->status, AgentStatus::Idle);
    EXPECT_EQ(state->position, Vec2(5.0f, 5.0f));
    EXPECT_EQ(state->velocity, Vec2(0.0f, 0.0f));
}

/// Regression: dropping a waypoint must follow real progress, not merely "the agent moved".
/// The straight line from start to goal leaves the L, so beelining wedges the agent.
TEST(SimulationTest, AgentRoutesAroundACornerAndArrives)
{
    const vwmini::NavMesh mesh = fixtures::make_mesh(fixtures::l_corridor());
    Simulation sim{vwmini::NavMesh{mesh}};
    const AgentId id =
        sim.add_agent(AgentConfig{Vec2{0.3f, 0.5f}, 0.2f, 1.0f, Vec2{1.5f, 2.7f}, 0.1f}).value();
    ASSERT_EQ(sim.agent(id)->status, AgentStatus::Moving);

    for (int index = 0; index < 200 && sim.agent(id)->status == AgentStatus::Moving; ++index)
    {
        ASSERT_TRUE(sim.step(0.05f).has_value());
        const auto state = sim.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_TRUE(mesh.contains(state->position))
            << "agent left the mesh at (" << state->position.x << "," << state->position.y << ")";
        EXPECT_LE(vwmini::length(state->velocity), 1.0f + 1.0e-4f);
    }
    const auto state = sim.agent(id);
    EXPECT_EQ(state->status, AgentStatus::Reached);
    EXPECT_LE(dist(*state->goal, state->position), 0.1f + 1.0e-3f);
}

/// Coarse substeps must not derail a route: the same corner is rounded with one big step.
TEST(SimulationTest, CoarseStepsStillRoundTheCorner)
{
    const vwmini::NavMesh mesh = fixtures::make_mesh(fixtures::l_corridor());
    Simulation sim{vwmini::NavMesh{mesh}};
    const AgentId id =
        sim.add_agent(AgentConfig{Vec2{0.3f, 0.5f}, 0.2f, 1.0f, Vec2{1.5f, 2.7f}, 0.1f}).value();

    ASSERT_TRUE(sim.step(2.0f).has_value());
    EXPECT_TRUE(mesh.contains(sim.agent(id)->position));
    ASSERT_TRUE(sim.step(10.0f).has_value());
    EXPECT_EQ(sim.agent(id)->status, AgentStatus::Reached);
    EXPECT_TRUE(mesh.contains(sim.agent(id)->position));
}

TEST(SimulationTest, CrossingAgentsAvoidAndStillArrive)
{
    const vwmini::NavMesh mesh = fixtures::make_mesh(fixtures::square_pair(10.0f));
    Simulation sim{vwmini::NavMesh{mesh}};
    const auto east =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.5f, 1.5f, Vec2{8.0f, 5.0f}, -1.0f}).value();
    const auto north =
        sim.add_agent(AgentConfig{Vec2{5.0f, 2.0f}, 0.5f, 1.5f, Vec2{5.0f, 8.0f}, -1.0f}).value();

    for (int index = 0; index < 200; ++index)
    {
        ASSERT_TRUE(sim.step(0.1f).has_value());
        expect_no_overlap(sim, east, north);
        expect_physical_invariants(sim, mesh, {east, north});
    }
    EXPECT_EQ(sim.agent(east)->status, AgentStatus::Reached);
    EXPECT_EQ(sim.agent(north)->status, AgentStatus::Reached);
}

TEST(SimulationTest, HeadOnAgentsPassWithoutOverlapping)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto left =
        sim.add_agent(AgentConfig{Vec2{3.0f, 5.0f}, 0.5f, 1.5f, Vec2{7.0f, 5.0f}, -1.0f}).value();
    const auto right =
        sim.add_agent(AgentConfig{Vec2{7.0f, 5.0f}, 0.5f, 1.5f, Vec2{3.0f, 5.0f}, -1.0f}).value();

    for (int index = 0; index < 100; ++index)
    {
        ASSERT_TRUE(sim.step(0.1f).has_value());
        expect_no_overlap(sim, left, right);
    }
    for (const AgentId id : {left, right})
    {
        const auto state = sim.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_TRUE(std::isfinite(state->position.x));
        EXPECT_TRUE(std::isfinite(state->position.y));
        EXPECT_LE(vwmini::length(state->velocity), 1.5f + 1.0e-4f);
    }
}

/// SIM-011 overtaking: a faster disc behind a slower one on the same heading must pass it in
/// open space without interpenetrating, and both must still reach their goals.
TEST(SimulationTest, OvertakingAgentsPassWithoutOverlapping)
{
    const vwmini::NavMesh mesh = fixtures::make_mesh(fixtures::square_pair(10.0f));
    Simulation sim{vwmini::NavMesh{mesh}};
    const AgentId slow =
        sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.3f, 0.8f, Vec2{8.0f, 5.0f}, -1.0f}).value();
    const AgentId fast =
        sim.add_agent(AgentConfig{Vec2{1.0f, 5.0f}, 0.3f, 2.0f, Vec2{9.0f, 5.0f}, -1.0f}).value();

    for (int index = 0; index < 300; ++index)
    {
        ASSERT_TRUE(sim.step(0.1f).has_value());
        expect_no_overlap(sim, slow, fast);
        expect_physical_invariants(sim, mesh, {slow, fast});
    }
    EXPECT_EQ(sim.agent(slow)->status, AgentStatus::Reached);
    EXPECT_EQ(sim.agent(fast)->status, AgentStatus::Reached);
}

TEST(SimulationTest, InitiallyOverlappingAgentsSeparate)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto first =
        sim.add_agent(AgentConfig{Vec2{5.0f, 5.0f}, 0.5f, 1.0f, Vec2{2.0f, 5.0f}, -1.0f}).value();
    const auto second =
        sim.add_agent(AgentConfig{Vec2{5.1f, 5.0f}, 0.5f, 1.0f, Vec2{8.0f, 5.0f}, -1.0f}).value();

    for (int index = 0; index < 50; ++index)
    {
        ASSERT_TRUE(sim.step(0.1f).has_value());
        const auto a = sim.agent(first);
        const auto b = sim.agent(second);
        EXPECT_TRUE(std::isfinite(a->position.x) && std::isfinite(b->position.x));
    }
    EXPECT_GE(dist(sim.agent(first)->position, sim.agent(second)->position), 0.9f);
}

TEST(SimulationTest, QueriesReturnIndependentSnapshots)
{
    auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
    const auto id = sim.add_agent(centered_config()).value();
    auto snapshot = sim.agent(id);
    ASSERT_TRUE(snapshot.has_value());
    snapshot->position = Vec2{0.0f, 0.0f};
    snapshot->status = AgentStatus::NoPath;

    EXPECT_EQ(sim.agent(id)->position, Vec2(5.0f, 5.0f));
    EXPECT_EQ(sim.agent(id)->status, AgentStatus::Idle);
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(SimulationTest, IdenticalSimulationsProduceIdenticalStates)
{
    const auto run = []
    {
        auto sim = Simulation(fixtures::make_mesh(fixtures::square_pair(10.0f)));
        const auto east =
            sim.add_agent(AgentConfig{Vec2{2.0f, 5.0f}, 0.5f, 1.5f, Vec2{8.0f, 5.0f}, -1.0f})
                .value();
        const AgentId north =
            sim.add_agent(AgentConfig{Vec2{5.0f, 2.0f}, 0.5f, 1.5f, Vec2{5.0f, 8.0f}, -1.0f})
                .value();
        for (int index = 0; index < 50; ++index)
        {
            EXPECT_TRUE(sim.step(0.07f).has_value());
        }
        const auto east_state = sim.agent(east);
        const auto north_state = sim.agent(north);
        return std::vector<float>{east_state->position.x,  east_state->position.y,
                                  east_state->velocity.x,  east_state->velocity.y,
                                  north_state->position.x, north_state->position.y,
                                  north_state->velocity.x, north_state->velocity.y};
    };

    EXPECT_EQ(run(), run());
}
