#include <vwmini/simulation.hpp>

#include "test_common.hpp"
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using namespace vwmini;

TEST(SimulationAddAgent, ValidAgentSucceeds)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{0.5f, 0.5f}});
    ASSERT_TRUE(id);
    EXPECT_EQ(id->value, 1u);
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(SimulationAddAgent, PositionOutsideMeshRejected)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{3.0f, 3.0f}});
    EXPECT_FALSE(id);
    EXPECT_EQ(id.error().code, ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationAddAgent, InvalidConfigRejected)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    // Non-positive radius.
    EXPECT_FALSE(sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.0f}));
    // Non-positive speed.
    EXPECT_FALSE(sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.25f, 0.0f}));
    // Negative arrival radius other than -1.
    EXPECT_FALSE(sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.25f, 1.0f, {}, -5.0f}));
    // Non-finite position.
    EXPECT_FALSE(sim.add_agent(AgentConfig{{std::numeric_limits<float>::quiet_NaN(), 0.5f}}));
    EXPECT_EQ(sim.agent_count(), 0u);
}

TEST(SimulationAddAgent, GoalOutsideMeshRejected)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.25f, 1.0f, Vec2{5.0f, 5.0f}});
    EXPECT_FALSE(id);
    EXPECT_EQ(id.error().code, ErrorCode::OutsideMesh);
}

TEST(SimulationRemoveAgent, ValidAndInvalidIds)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{0.5f, 0.5f}});
    ASSERT_TRUE(id);
    EXPECT_TRUE(sim.remove_agent(*id));
    EXPECT_EQ(sim.agent_count(), 0u);
    EXPECT_EQ(sim.agent(*id), std::nullopt); // removed agent is not live
    // id.value == 0 is invalid.
    EXPECT_FALSE(sim.remove_agent(AgentId{0}));
    // Unknown id.
    EXPECT_FALSE(sim.remove_agent(AgentId{99}));
}

TEST(SimulationStep, IdlesToGoalInsideMesh)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.25f, 1.0f, Vec2{1.5f, 0.5f}});
    ASSERT_TRUE(id);

    const auto before = sim.agent(*id);
    ASSERT_TRUE(before);
    EXPECT_EQ(before->status, AgentStatus::Moving);

    // Goal is directly reachable in one cell; stepping should make progress.
    float prev_dist = length(before->position - Vec2{1.5f, 0.5f});
    Vec2 pos = before->position;
    bool reached = false;
    for (int i = 0; i < 20; ++i) {
        ASSERT_TRUE(sim.step(0.1f));
        const auto st = sim.agent(*id);
        ASSERT_TRUE(st);
        const float d = length(st->position - Vec2{1.5f, 0.5f});
        EXPECT_LE(d, prev_dist + 1e-3f); // monotonic (or arrived)
        prev_dist = d;
        pos = st->position;
        if (st->status == AgentStatus::Reached) {
            reached = true;
            break;
        }
    }
    EXPECT_TRUE(reached);
    const auto after = sim.agent(*id);
    ASSERT_TRUE(after);
    EXPECT_EQ(after->status, AgentStatus::Reached);
    EXPECT_LE(length(after->position - Vec2{1.5f, 0.5f}), 0.25f);
}

TEST(SimulationStep, NoGoalStaysIdleAndStill)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{0.5f, 0.5f}});
    ASSERT_TRUE(id);
    const auto before = sim.agent(*id);
    ASSERT_TRUE(before);
    EXPECT_EQ(before->status, AgentStatus::Idle);
    ASSERT_TRUE(sim.step(0.2f));
    const auto after = sim.agent(*id);
    ASSERT_TRUE(after);
    EXPECT_EQ(after->status, AgentStatus::Idle);
    EXPECT_EQ(after->position, before->position);
    EXPECT_EQ(after->velocity, (Vec2{0.0f, 0.0f}));
}

TEST(SimulationStep, ReachedAgentStops)
{
    const NavMesh m = test::make_large_triangle();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{2.0f, 2.0f}, 0.25f, 1.4f, Vec2{18.0f, 2.0f}});
    ASSERT_TRUE(id);
    // Step until reached (single-cell mesh, straight-line route).
    bool reached = false;
    for (int i = 0; i < 500; ++i) {
        ASSERT_TRUE(sim.step(0.1f));
        const auto st = sim.agent(*id);
        ASSERT_TRUE(st);
        if (st->status == AgentStatus::Reached) {
            reached = true;
            break;
        }
    }
    ASSERT_TRUE(reached);
    const auto st = sim.agent(*id);
    ASSERT_TRUE(st);
    EXPECT_EQ(st->status, AgentStatus::Reached);
    EXPECT_EQ(st->velocity, (Vec2{0.0f, 0.0f}));
}

TEST(SimulationStep, InvalidDurationRejected)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    ASSERT_TRUE(sim.add_agent(AgentConfig{{0.5f, 0.5f}}));
    EXPECT_FALSE(sim.step(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(sim.step(-0.1f));
}

TEST(SimulationAddAgent, FailedAddLeavesNoAgentAndReusesFirstId)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    // Goal outside the mesh: must be rejected WITHOUT creating an agent.
    const auto bad = sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.25f, 1.0f, Vec2{5.0f, 5.0f}});
    EXPECT_FALSE(bad);
    EXPECT_EQ(bad.error().code, ErrorCode::OutsideMesh);
    EXPECT_EQ(sim.agent_count(), 0u);
    // A subsequent valid add must receive id 1 (no leak, ids are durable).
    const auto good = sim.add_agent(AgentConfig{{0.5f, 0.5f}});
    ASSERT_TRUE(good);
    EXPECT_EQ(good->value, 1u);
    EXPECT_EQ(sim.agent_count(), 1u);
}

TEST(SimulationRemoveAgent, RemovedIdStaysInvalidAfterReAdd)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto a = sim.add_agent(AgentConfig{{0.5f, 0.5f}});
    ASSERT_TRUE(a);
    EXPECT_EQ(a->value, 1u);
    ASSERT_TRUE(sim.remove_agent(*a));
    // Add again: gets a fresh, larger id; the old id must stay invalid.
    const auto b = sim.add_agent(AgentConfig{{1.5f, 1.5f}});
    ASSERT_TRUE(b);
    EXPECT_EQ(b->value, 2u);
    EXPECT_EQ(sim.agent_count(), 1u);
    EXPECT_EQ(sim.agent(*a), std::nullopt); // removed id invalid
    EXPECT_FALSE(sim.remove_agent(*a));     // stale id -> error
    EXPECT_FALSE(sim.set_goal(*a, Vec2{1.0f, 1.0f}));
    const auto st = sim.agent(*b);
    ASSERT_TRUE(st);
    EXPECT_EQ(st->status, AgentStatus::Idle);
}

TEST(SimulationAvoidance, CoincidentCentresSeparateDeterministically)
{
    const NavMesh m = test::make_large_triangle();
    Simulation sim(m);
    const auto a = sim.add_agent(AgentConfig{Vec2{10.0f, 10.0f}, 0.5f, 2.0f, Vec2{18.0f, 2.0f}});
    const auto b = sim.add_agent(AgentConfig{Vec2{10.0f, 10.0f}, 0.5f, 2.0f, Vec2{2.0f, 18.0f}});
    ASSERT_TRUE(a);
    ASSERT_TRUE(b);

    bool ok = true;
    for (int i = 0; i < 40; ++i) {
        if (!sim.step(0.1f)) {
            ok = false;
            break;
        }
    }
    EXPECT_TRUE(ok);
    const auto sa = sim.agent(*a);
    const auto sb = sim.agent(*b);
    ASSERT_TRUE(sa);
    ASSERT_TRUE(sb);
    // They never overlap (each radius 0.5 => centres >= 1.0).
    EXPECT_GE(length(sa->position - sb->position), 1.0f - 1e-2f);
    // Each has made meaningful progress away from the shared start.
    EXPECT_GE(length(sa->position - Vec2{10.0f, 10.0f}), 0.5f);
    EXPECT_GE(length(sb->position - Vec2{10.0f, 10.0f}), 0.5f);
}

TEST(SimulationStep, LargeFiniteDurationIsSafeAndReachesGoal)
{
    const NavMesh m = test::make_square();
    Simulation sim(m);
    const auto id = sim.add_agent(AgentConfig{{0.5f, 0.5f}, 0.25f, 1.0f, Vec2{1.5f, 0.5f}});
    ASSERT_TRUE(id);
    // FLT_MAX would previously overflow seconds/0.05 -> ceil -> size_t (UB).
    const auto r = sim.step(std::numeric_limits<float>::max());
    ASSERT_TRUE(r);
    const auto st = sim.agent(*id);
    ASSERT_TRUE(st);
    EXPECT_EQ(st->status, AgentStatus::Reached);
    EXPECT_LE(length(st->position - Vec2{1.5f, 0.5f}), 0.25f);
}

TEST(SimulationAvoidance, CrossingAgentsDoNotOverlap)
{
    const NavMesh m = test::make_grid();
    Simulation sim(m);
    const auto a = sim.add_agent(AgentConfig{Vec2{0.3f, 1.95f}, 0.3f, 2.0f, Vec2{3.7f, 1.95f}});
    const auto b = sim.add_agent(AgentConfig{Vec2{1.95f, 0.3f}, 0.3f, 2.0f, Vec2{1.95f, 3.7f}});
    ASSERT_TRUE(a);
    ASSERT_TRUE(b);

    bool progressed_a = false, progressed_b = false;
    for (int i = 0; i < 30; ++i) {
        ASSERT_TRUE(sim.step(0.1f));
        const auto sa = sim.agent(*a);
        const auto sb = sim.agent(*b);
        ASSERT_TRUE(sa);
        ASSERT_TRUE(sb);
        // Never overlap (with a small tolerance for float noise).
        EXPECT_GE(length(sa->position - sb->position), 0.6f - 1e-2f);
        if (sa->position.x > 1.5f) {
            progressed_a = true;
        }
        if (sb->position.y > 1.5f) {
            progressed_b = true;
        }
    }
    EXPECT_TRUE(progressed_a);
    EXPECT_TRUE(progressed_b);
}
