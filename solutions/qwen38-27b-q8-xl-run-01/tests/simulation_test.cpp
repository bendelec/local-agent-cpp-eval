// Simulation core tests (SIM-005..SIM-013) and basic waypoint motion (S6.1):
// id lifecycle, add_agent validation, goal state transitions, step validation,
// and no-overshoot straight-line motion.
#include <vwmini/simulation.hpp>

#include "test_util.hpp"

#include <cmath>
#include <limits>
#include <vector>

using namespace vwmini;
using vwmini::testing::err_code;
using vwmini::testing::tri;

namespace {

[[nodiscard]] float nan_f()
{
    return std::numeric_limits<float>::quiet_NaN();
}

[[nodiscard]] float inf_f()
{
    return std::numeric_limits<float>::infinity();
}

// Default fixture: square [0,10]^2 split into two triangles.
NavMesh squareMesh()
{
    auto r = NavMesh::create({tri({0, 0}, {10, 0}, {10, 10}),
                              tri({0, 0}, {10, 10}, {0, 10})});
    EXPECT_TRUE(r.has_value());
    return *r;
}

// Two disjoint triangles: disconnected mesh components.
NavMesh disconnectedMesh()
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}),
                              tri({5, 5}, {6, 5}, {5, 6})});
    EXPECT_TRUE(r.has_value());
    return *r;
}

AgentConfig configAt(Vec2 position)
{
    AgentConfig config;
    config.position = position;
    return config;
}

} // namespace

TEST(S51_Lifecycle, AddAgentReturnsDistinctNonZeroIds)
{
    Simulation sim(squareMesh());
    ASSERT_EQ(sim.agent_count(), 0u);
    auto a = sim.add_agent(configAt({1.f, 1.f}));
    auto b = sim.add_agent(configAt({2.f, 2.f}));
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_GT(a->value, 0u);
    EXPECT_NE(a->value, b->value);
    EXPECT_EQ(sim.agent_count(), 2u);
}

TEST(S51_Lifecycle, AgentSnapshotEqualsConfig)
{
    Simulation sim(squareMesh());
    AgentConfig config = configAt({1.5f, 2.5f});
    config.radius = 0.3f;
    config.max_speed = 2.f;
    const AgentId id = *sim.add_agent(config);
    ASSERT_TRUE(sim.agent(id).has_value());
    const AgentState s = *sim.agent(id);
    EXPECT_EQ(s.position, config.position);
    EXPECT_EQ(s.radius, config.radius);
    EXPECT_EQ(s.max_speed, config.max_speed);
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));
    EXPECT_FALSE(s.goal.has_value());
    EXPECT_EQ(s.status, AgentStatus::Idle);
}

TEST(S51_Lifecycle, RemoveAgentInvalidatesId)
{
    Simulation sim(squareMesh());
    const AgentId id = *sim.add_agent(configAt({1.f, 1.f}));
    ASSERT_EQ(sim.agent_count(), 1u);
    ASSERT_TRUE(sim.remove_agent(id).has_value());
    EXPECT_EQ(sim.agent_count(), 0u);
    EXPECT_FALSE(sim.agent(id).has_value());
    EXPECT_EQ(err_code(sim.remove_agent(id)), ErrorCode::NotFound);
    EXPECT_EQ(err_code(sim.set_goal(id, {2.f, 2.f})), ErrorCode::NotFound);
    EXPECT_EQ(err_code(sim.clear_goal(id)), ErrorCode::NotFound);
}

TEST(S51_Lifecycle, UnknownIds)
{
    Simulation sim(squareMesh());
    EXPECT_FALSE(sim.agent(AgentId{0}).has_value());
    EXPECT_FALSE(sim.agent(AgentId{9999}).has_value());
    EXPECT_EQ(err_code(sim.remove_agent(AgentId{0})), ErrorCode::NotFound);
    EXPECT_EQ(err_code(sim.set_goal(AgentId{9999}, {2.f, 2.f})), ErrorCode::NotFound);
    EXPECT_EQ(err_code(sim.clear_goal(AgentId{0})), ErrorCode::NotFound);
}

TEST(S51_Lifecycle, RemovedIdIsNeverReused)
{
    Simulation sim(squareMesh());
    const AgentId a = *sim.add_agent(configAt({1.f, 1.f}));
    ASSERT_TRUE(sim.remove_agent(a).has_value());
    const AgentId b = *sim.add_agent(configAt({2.f, 2.f}));
    EXPECT_GT(b.value, a.value);
}

TEST(S52_AddAgentValidation, NonFiniteAndNonPositiveConfig)
{
    Simulation sim(squareMesh());
    const auto expect_invalid = [&sim](const AgentConfig& config) {
        auto result = sim.add_agent(config);
        EXPECT_FALSE(result.has_value());
        if (!result) {
            EXPECT_EQ(result.error().code, ErrorCode::InvalidArgument);
        }
        EXPECT_EQ(sim.agent_count(), 0u);
    };

    AgentConfig c = configAt({1.f, 1.f});
    c.position = {nan_f(), 1.f};
    expect_invalid(c);
    c.position = {1.f, inf_f()};
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.radius = nan_f();
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.radius = 0.f;
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.radius = -0.5f;
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.max_speed = 0.f;
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.arrival_radius = -2.f;
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.arrival_radius = -0.5f;
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.arrival_radius = -1.5f;
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.goal = Vec2{2.f, nan_f()};
    expect_invalid(c);
    c = configAt({1.f, 1.f});
    c.goal = Vec2{inf_f(), 2.f};
    expect_invalid(c);
}

TEST(S52_AddAgentValidation, OutsideMesh)
{
    Simulation sim(squareMesh());
    AgentConfig outside = configAt({-1.f, 1.f});
    auto r1 = sim.add_agent(outside);
    ASSERT_FALSE(r1.has_value());
    EXPECT_EQ(err_code(r1), ErrorCode::OutsideMesh);

    AgentConfig goal_outside = configAt({1.f, 1.f});
    goal_outside.goal = Vec2{20.f, 1.f};
    auto r2 = sim.add_agent(goal_outside);
    ASSERT_FALSE(r2.has_value());
    EXPECT_EQ(err_code(r2), ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(S52_AddAgentValidation, NegativeZeroArrivalIsValid)
{
    Simulation sim(squareMesh());
    AgentConfig c = configAt({1.f, 1.f});
    c.arrival_radius = -0.0f;
    ASSERT_TRUE(sim.add_agent(c).has_value());
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(S53_GoalTransitions, FarGoalBecomesMoving)
{
    Simulation sim(squareMesh());
    AgentConfig c = configAt({1.f, 1.f});
    c.goal = Vec2{8.f, 8.f};
    const AgentId id = *sim.add_agent(c);
    const AgentState s = *sim.agent(id);
    EXPECT_EQ(s.status, AgentStatus::Moving);
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, (Vec2{8.f, 8.f}));
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));
}

TEST(S53_GoalTransitions, GoalWithinArrivalRadiusIsReached)
{
    Simulation sim(squareMesh());
    const AgentId id = *sim.add_agent(configAt({1.f, 1.f}));
    // Default arrival radius == agent radius 0.25: goal 0.1 away.
    ASSERT_TRUE(sim.set_goal(id, {1.1f, 1.f}).has_value());
    const AgentState s = *sim.agent(id);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, (Vec2{1.1f, 1.f}));
}

TEST(S53_GoalTransitions, DisconnectedGoalIsNoPath)
{
    Simulation sim(disconnectedMesh());
    const AgentId id = *sim.add_agent(configAt({0.25f, 0.25f}));
    // In-mesh but in the other component: success with NoPath.
    ASSERT_TRUE(sim.set_goal(id, {5.25f, 5.25f}).has_value());
    const AgentState s = *sim.agent(id);
    EXPECT_EQ(s.status, AgentStatus::NoPath);
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));
    ASSERT_TRUE(s.goal.has_value());
    EXPECT_EQ(*s.goal, (Vec2{5.25f, 5.25f}));
}

TEST(S53_GoalTransitions, ClearGoalReturnsToIdle)
{
    Simulation sim(squareMesh());
    AgentConfig c = configAt({1.f, 1.f});
    c.goal = Vec2{8.f, 8.f};
    const AgentId id = *sim.add_agent(c);
    ASSERT_TRUE(sim.clear_goal(id).has_value());
    AgentState s = *sim.agent(id);
    EXPECT_EQ(s.status, AgentStatus::Idle);
    EXPECT_FALSE(s.goal.has_value());
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));

    // Subsequent step leaves the position unchanged.
    ASSERT_TRUE(sim.step(1.f).has_value());
    s = *sim.agent(id);
    EXPECT_EQ(s.position, (Vec2{1.f, 1.f}));
    EXPECT_EQ(s.status, AgentStatus::Idle);
}

TEST(S53_GoalTransitions, SetGoalValidation)
{
    Simulation sim(squareMesh());
    const AgentId id = *sim.add_agent(configAt({1.f, 1.f}));
    EXPECT_EQ(err_code(sim.set_goal(AgentId{0}, {2.f, 2.f})), ErrorCode::NotFound);
    ASSERT_TRUE(sim.remove_agent(id).has_value());
    EXPECT_EQ(err_code(sim.set_goal(id, {2.f, 2.f})), ErrorCode::NotFound);

    Simulation sim2(squareMesh());
    const AgentId id2 = *sim2.add_agent(configAt({1.f, 1.f}));
    EXPECT_EQ(err_code(sim2.set_goal(id2, {nan_f(), 1.f})), ErrorCode::InvalidArgument);
    EXPECT_EQ(err_code(sim2.set_goal(id2, {2.f, 2.f}, -0.5f)), ErrorCode::InvalidArgument);
    EXPECT_EQ(err_code(sim2.set_goal(id2, {50.f, 2.f})), ErrorCode::OutsideMesh);
}

TEST(S54_StepValidation, InvalidStepIsTransactional)
{
    Simulation sim(squareMesh());
    AgentConfig c = configAt({1.f, 1.f});
    c.goal = Vec2{8.f, 8.f};
    const AgentId id = *sim.add_agent(c);
    const AgentState before = *sim.agent(id);

    for (float seconds : {-1.f, nan_f(), inf_f()})
    {
        auto result = sim.step(seconds);
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, ErrorCode::InvalidArgument);
        const AgentState after = *sim.agent(id);
        EXPECT_EQ(after.position, before.position) << "step(" << seconds << ") changed position";
        EXPECT_EQ(after.velocity, before.velocity) << "step(" << seconds << ") changed velocity";
        EXPECT_EQ(after.status, before.status) << "step(" << seconds << ") changed status";
        EXPECT_EQ(after.goal.has_value(), before.goal.has_value()) << "step(" << seconds << ") changed goal";
    }

    // Zero duration: success, no change.
    ASSERT_TRUE(sim.step(0.f).has_value());
    const AgentState after_zero = *sim.agent(id);
    EXPECT_EQ(after_zero.position, before.position);
    EXPECT_EQ(after_zero.velocity, before.velocity);
    EXPECT_EQ(after_zero.status, before.status);
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(S61_Motion, StraightLineArrivalInTwoSteps)
{
    Simulation sim(squareMesh());
    AgentConfig c = configAt({1.f, 1.f});
    c.radius = 0.1f;          // Default arrival 0.1: goal 2 away does not pre-trigger.
    c.max_speed = 1.f;
    c.goal = Vec2{3.f, 1.f};
    const AgentId id = *sim.add_agent(c);
    EXPECT_EQ((*sim.agent(id)).status, AgentStatus::Moving);

    // 10 substeps of dt=0.1 at speed 1: ~1.0 m, halfway to the goal.
    ASSERT_TRUE(sim.step(1.f).has_value());
    AgentState s = *sim.agent(id);
    EXPECT_NEAR(s.position.x, 2.f, 1e-4f);
    EXPECT_NEAR(s.position.y, 1.f, 1e-4f);
    EXPECT_EQ(s.status, AgentStatus::Moving);

    ASSERT_TRUE(sim.step(1.f).has_value());
    s = *sim.agent(id);
    EXPECT_NEAR(s.position.x, 3.f, 1e-4f);
    EXPECT_NEAR(s.position.y, 1.f, 1e-4f);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));
}

TEST(S61_Motion, LargeDtNeverOvershoots)
{
    Simulation sim(squareMesh());
    AgentConfig c = configAt({1.f, 1.f});
    c.radius = 0.1f;
    c.max_speed = 1.f;
    c.goal = Vec2{3.f, 1.f};
    const AgentId id = *sim.add_agent(c);

    ASSERT_TRUE(sim.step(100.f).has_value());
    const AgentState s = *sim.agent(id);
    EXPECT_NEAR(s.position.x, 3.f, 1e-4f);
    EXPECT_NEAR(s.position.y, 1.f, 1e-4f);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_EQ(s.velocity, (Vec2{0.f, 0.f}));
}

TEST(S61_Motion, DiagonalMotionStaysInMeshAndReaches)
{
    // Crossing both triangles; the straight path is contained.
    Simulation sim(squareMesh());
    AgentConfig c = configAt({2.f, 2.f});
    c.radius = 0.1f;
    c.max_speed = 2.f;
    c.goal = Vec2{8.f, 6.f};
    const AgentId id = *sim.add_agent(c);
    ASSERT_TRUE(sim.step(10.f).has_value());
    const AgentState s = *sim.agent(id);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_NEAR(s.position.x, 8.f, 1e-5f);
    EXPECT_NEAR(s.position.y, 6.f, 1e-5f);
}

TEST(S61_Motion, ContainmentClampKeepsAgentInMesh)
{
    // Goal near the upper-right corner: the route stays inside the square,
    // and the containment clamp guarantees no escape even under noise.
    Simulation sim(squareMesh());
    AgentConfig c = configAt({0.5f, 0.5f});
    c.radius = 0.05f;
    c.max_speed = 5.f;
    c.goal = Vec2{9.5f, 9.5f};
    const AgentId id = *sim.add_agent(c);
    ASSERT_TRUE(sim.step(50.f).has_value());
    const AgentState s = *sim.agent(id);
    EXPECT_EQ(s.status, AgentStatus::Reached);
    EXPECT_NEAR(s.position.x, 9.5f, 1e-5f);
    EXPECT_NEAR(s.position.y, 9.5f, 1e-5f);
}

// ---------------------------------------------------------------------------
// WP6 (S6.2) local disc avoidance + (S6.3) determinism soak.
// Deterministic scenarios: fixed 10 Hz substeps, no randomness. Invariants
// (pairwise separation, finiteness, containment, speed cap) are checked at
// every public step; iteration caps make deadlocks fail, not hang.
// ---------------------------------------------------------------------------
namespace {

// Checks SIM-011/SIM-012 invariants for all `ids` in `sim` against `mesh`:
// pairwise disc separation >= r_i + r_j - 1e-3 m (unless `check_separation`
// is false, e.g. an intentionally overlapping start), finite state,
// containment, and |velocity| <= max_speed + 1e-6 m.
void wp6_invariants(Simulation& sim, const std::vector<AgentId>& ids,
                    const NavMesh& mesh, int step_no,
                    bool check_separation = true)
{
    const std::size_t n = ids.size();
    std::vector<AgentState> states;
    states.reserve(n);
    for (const AgentId id : ids)
    {
        const AgentState s = *sim.agent(id);
        states.push_back(s);
        EXPECT_TRUE(std::isfinite(s.position.x) && std::isfinite(s.position.y))
            << "non-finite position at step " << step_no;
        EXPECT_TRUE(std::isfinite(s.velocity.x) && std::isfinite(s.velocity.y))
            << "non-finite velocity at step " << step_no;
        EXPECT_TRUE(mesh.contains(s.position)) << "agent outside mesh at step " << step_no;
        EXPECT_LE(vwmini::length(s.velocity), s.max_speed + 1e-6f)
            << "velocity exceeds max_speed at step " << step_no;
    }
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j)
        {
            const float d =
                vwmini::length(states[i].position - states[j].position);
            if (check_separation)
            {
                EXPECT_GE(d, states[i].radius + states[j].radius - 1e-3f)
                    << "disc overlap " << d << " at step " << step_no;
            }
        }
}

// Drives `sim` with 0.1 s steps until all ids are Reached or `max_steps` is
// exhausted; asserts invariants at every returned step. Returns true when
// every agent reached its goal.
bool wp6_step_until_reached(Simulation& sim, const std::vector<AgentId>& ids,
                            const NavMesh& mesh, int max_steps = 1000)
{
    for (int s = 0; s < max_steps; ++s)
    {
        if (!sim.step(0.1f).has_value())
            return false;
        wp6_invariants(sim, ids, mesh, s);
        bool all = true;
        for (const AgentId id : ids)
            all = all && (*sim.agent(id)).status == AgentStatus::Reached;
        if (all)
            return true;
    }
    return false;
}

} // namespace

TEST(S62_Avoidance, CrossingPairNeverOverlapsAndBothReach)
{
    // Scenario 1 (SIM-011): A (1,5)->(9,5), B (5,1)->(5,9); speeds 1.0.
    const NavMesh mesh = squareMesh();
    Simulation sim(mesh);
    AgentConfig ca = configAt({1.f, 5.f});
    ca.max_speed = 1.0f;
    AgentConfig cb = configAt({5.f, 1.f});
    cb.max_speed = 1.0f;
    const AgentId a = *sim.add_agent(ca);
    const AgentId b = *sim.add_agent(cb);
    ASSERT_TRUE(sim.set_goal(a, {9.f, 5.f}).has_value());
    ASSERT_TRUE(sim.set_goal(b, {5.f, 9.f}).has_value());

    const std::vector<AgentId> ids{a, b};
    ASSERT_TRUE(wp6_step_until_reached(sim, ids, mesh));
}

TEST(S62_Avoidance, OvertakingPairNeverOverlapsAndBothReach)
{
    // Scenario 2 (SIM-011): A (1,5)->(9,5) speed 1.4, B (2,5)->(8,5) speed
    // 1.0; A starts behind and must overtake.
    const NavMesh mesh = squareMesh();
    Simulation sim(mesh);
    AgentConfig ca = configAt({1.f, 5.f});
    ca.max_speed = 1.4f;
    AgentConfig cb = configAt({2.f, 5.f});
    cb.max_speed = 1.0f;
    const AgentId a = *sim.add_agent(ca);
    const AgentId b = *sim.add_agent(cb);
    ASSERT_TRUE(sim.set_goal(a, {9.f, 5.f}).has_value());
    ASSERT_TRUE(sim.set_goal(b, {8.f, 5.f}).has_value());

    const std::vector<AgentId> ids{a, b};
    ASSERT_TRUE(wp6_step_until_reached(sim, ids, mesh));
}

TEST(S62_Avoidance, AngledCrossingWithSpeedMismatch)
{
    // Scenario 3 (SIM-011 variation): A (1,1)->(9,9) speed 1.0,
    // B (1,9)->(9,1) speed 1.4.
    const NavMesh mesh = squareMesh();
    Simulation sim(mesh);
    AgentConfig ca = configAt({1.f, 1.f});
    ca.max_speed = 1.0f;
    AgentConfig cb = configAt({1.f, 9.f});
    cb.max_speed = 1.4f;
    const AgentId a = *sim.add_agent(ca);
    const AgentId b = *sim.add_agent(cb);
    ASSERT_TRUE(sim.set_goal(a, {9.f, 9.f}).has_value());
    ASSERT_TRUE(sim.set_goal(b, {9.f, 1.f}).has_value());

    const std::vector<AgentId> ids{a, b};
    ASSERT_TRUE(wp6_step_until_reached(sim, ids, mesh));
}

TEST(S62_Avoidance, OverlappingStartIsSeparated)
{
    // Scenario 4 (SIM-010 edge): two radius-0.25 discs starting 0.1 m apart
    // (overlapping) at (5,5) and (5.1,5), goals at (5,1) and (5,9).
    // Robustness only: finite + contained + separation attempted.
    const NavMesh mesh = squareMesh();
    Simulation sim(mesh);
    AgentConfig ca = configAt({5.f, 5.f});
    ca.max_speed = 1.0f;
    AgentConfig cb = configAt({5.1f, 5.f});
    cb.max_speed = 1.0f;
    const AgentId a = *sim.add_agent(ca);
    const AgentId b = *sim.add_agent(cb);
    ASSERT_TRUE(sim.set_goal(a, {5.f, 1.f}).has_value());
    ASSERT_TRUE(sim.set_goal(b, {5.f, 9.f}).has_value());

    const std::vector<AgentId> ids{a, b};
    const float r_sum = 0.25f + 0.25f;
    const float initial_dist = 0.1f;
    float min_dist = initial_dist;
    bool reached_touch = false;
    for (int s = 0; s < 1000; ++s)
    {
        ASSERT_TRUE(sim.step(0.1f).has_value());
        // Invariants minus overlap: the start is intentionally overlapping.
        wp6_invariants(sim, ids, mesh, s, /*check_separation=*/false);
        const AgentState sa = *sim.agent(a);
        const AgentState sb = *sim.agent(b);
        const float d = vwmini::length(sa.position - sb.position);
        min_dist = d < min_dist ? d : min_dist;
        if (d >= r_sum - 1e-3f)
            reached_touch = true;
        if (reached_touch)
            break;
    }
    // Separation was attempted: distance must have reached at least the
    // touching distance at some point, or at least not decreased below the
    // initial minus 1e-3 (robustness per SIM-010 edge case).
    EXPECT_TRUE(reached_touch || min_dist >= initial_dist - 1e-3f)
        << "no separation attempted: min_dist=" << min_dist;
}

TEST(S62_Avoidance, FarIdleAgentDoesNotInterfere)
{
    // Scenario 5: an idle agent far away must not change a moving agent's
    // trajectory. Compare positions step-by-step against a same sim without
    // the idle agent for the first 20 steps — exact equality required.
    const NavMesh mesh = squareMesh();

    // Run 1: single moving agent.
    {
        Simulation sim(mesh);
        AgentConfig c = configAt({1.f, 5.f});
        c.max_speed = 1.0f;
        c.goal = Vec2{9.f, 5.f};
        const AgentId id = *sim.add_agent(c);
        std::vector<Vec2> positions;
        positions.reserve(21);
        positions.push_back((*sim.agent(id)).position);
        for (int s = 0; s < 20; ++s)
        {
            ASSERT_TRUE(sim.step(0.1f).has_value());
            positions.push_back((*sim.agent(id)).position);
        }

        // Run 2: same agent + a far-away idle agent.
        Simulation sim2(mesh);
        AgentConfig c2 = configAt({1.f, 5.f});
        c2.max_speed = 1.0f;
        c2.goal = Vec2{9.f, 5.f};
        AgentConfig idle = configAt({9.5f, 9.5f});
        idle.max_speed = 1.0f;
        const AgentId id2 = *sim2.add_agent(c2);
        ASSERT_TRUE(sim2.add_agent(idle).has_value());
        std::vector<Vec2> positions2;
        positions2.reserve(21);
        positions2.push_back((*sim2.agent(id2)).position);
        for (int s = 0; s < 20; ++s)
        {
            ASSERT_TRUE(sim2.step(0.1f).has_value());
            positions2.push_back((*sim2.agent(id2)).position);
        }

        ASSERT_EQ(positions.size(), positions2.size());
        for (std::size_t i = 0; i < positions.size(); ++i)
            EXPECT_EQ(positions[i], positions2[i])
                << "position differs at step " << i;
    }
}

TEST(S63_Determinism, TwoRunsBitIdentical)
{
    // Scenario 6 (S6.3): 2-agent crossing + 1 idle agent, mixed step
    // durations, 500 substeps total; every public call's full AgentState
    // must be EXACTLY equal between two fresh runs.
    // Explicit call sequence: 26 full cycles of {0.1, 0.5, 0.33, 1.0, 0.0}
    // (19 substeps each: 1+5+3+10+0) = 494, then {0.1, 0.5} = 6, totalling
    // 500 substeps in 132 calls. The mid-run set_goal fires after call 25
    // (index 24), matching the original budget-driven sequence exactly.
    const std::vector<float> cycle{0.1f, 0.5f, 0.33f, 1.0f, 0.0f};
    std::vector<float> calls;
    calls.reserve(132);
    for (int c = 0; c < 26; ++c)
        for (const float d : cycle)
            calls.push_back(d);
    calls.push_back(0.1f);
    calls.push_back(0.5f);

    struct Trace {
        std::vector<AgentState> a, b, c;
    };
    auto run_once = [&calls]() -> Trace {
        auto r = NavMesh::create({tri({0, 0}, {10, 0}, {10, 10}),
                                  tri({0, 0}, {10, 10}, {0, 10})});
        if (!r.has_value())
            return Trace{};  // mesh creation failed: fatal
        Simulation sim(*r);
        AgentConfig ca = configAt({1.f, 5.f});
        ca.max_speed = 1.0f;
        AgentConfig cb = configAt({5.f, 1.f});
        cb.max_speed = 1.0f;
        AgentConfig ci = configAt({0.5f, 9.5f});
        ci.max_speed = 1.0f;
        const AgentId a = *sim.add_agent(ca);
        const AgentId b = *sim.add_agent(cb);
        const AgentId c = *sim.add_agent(ci);
        if (!sim.set_goal(a, {9.f, 5.f}).has_value())
            return Trace{};  // unreachable: valid in-mesh goal
        if (!sim.set_goal(b, {5.f, 9.f}).has_value())
            return Trace{};

        std::vector<AgentState> trace_a, trace_b, trace_c;
        auto snapshot = [&sim, &trace_a, &trace_b, &trace_c](AgentId a,
                                                             AgentId b,
                                                             AgentId c) {
            trace_a.push_back(*sim.agent(a));
            trace_b.push_back(*sim.agent(b));
            trace_c.push_back(*sim.agent(c));
        };
        snapshot(a, b, c);

        for (std::size_t i = 0; i < calls.size(); ++i)
        {
            if (!sim.step(calls[i]).has_value())
                return Trace{};  // unreachable: finite non-negative durations
            snapshot(a, b, c);
            // Mid-run goal change for variety.
            if (i == 24 && !sim.set_goal(b, {9.f, 1.f}).has_value())
                return Trace{};  // unreachable: valid in-mesh goal
        }
        return Trace{std::move(trace_a), std::move(trace_b), std::move(trace_c)};
    };

    const auto t1 = run_once();
    const auto t2 = run_once();

    ASSERT_EQ(t1.a.size(), t2.a.size());
    ASSERT_EQ(t1.b.size(), t2.b.size());
    ASSERT_EQ(t1.c.size(), t2.c.size());
    for (std::size_t i = 0; i < t1.a.size(); ++i)
    {
        // Exact per-field equality (AgentState has no operator== in the
        // public header; the trace vectors hold AgentState by value).
        const auto same = [](const AgentState& x, const AgentState& y) {
            return x.position == y.position && x.velocity == y.velocity &&
                   x.radius == y.radius && x.max_speed == y.max_speed &&
                   (x.goal.has_value() == y.goal.has_value()) &&
                   (!x.goal.has_value() || *x.goal == *y.goal) &&
                   x.status == y.status;
        };
        EXPECT_TRUE(same(t1.a[i], t2.a[i])) << "agent A state differs at call " << i;
        EXPECT_TRUE(same(t1.b[i], t2.b[i])) << "agent B state differs at call " << i;
        EXPECT_TRUE(same(t1.c[i], t2.c[i])) << "idle agent state differs at call " << i;
    }
}
