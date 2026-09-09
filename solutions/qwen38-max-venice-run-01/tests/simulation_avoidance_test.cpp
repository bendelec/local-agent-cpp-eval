#include <vwmini/simulation.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

// Local avoidance between disc agents (SIM-010..012): snapshot-based
// detours, no overlap of separated agents, active separation of initially
// overlapping agents, and bitwise-deterministic replays.

namespace {

using namespace vwmini::test;
using vwmini::AgentConfig;
using vwmini::AgentId;
using vwmini::AgentState;
using vwmini::AgentStatus;
using vwmini::NavMesh;
using vwmini::Simulation;
using vwmini::Vec2;

constexpr float kDt = 1.0f / 60.0f;
constexpr float kRadius = 0.3f;
constexpr float kRadiusSum = 2.0f * kRadius;
/// SIM-011 allows a 1e-3 tolerance below the radius sum.
constexpr float kEnforce = kRadiusSum - 1e-3f;
/// Separation is only enforced once the pair has clearly achieved it, so
/// initially overlapping agents get their separation phase first.
constexpr float kLatch = kRadiusSum + 0.02f;
/// 30 s at 60 Hz; all scenarios finish well within this budget.
constexpr std::size_t kMaxSteps = 1800;

[[nodiscard]] float distance(Vec2 a, Vec2 b) noexcept
{
    return length(a - b);
}

/// One big open triangle: straight routes fit with ample detour room.
[[nodiscard]] NavMesh open_mesh()
{
    return make_mesh({Tri(V(0.0f, 0.0f), V(20.0f, 0.0f), V(10.0f, 16.0f))});
}

/// A corridor too narrow for two radius-0.3 discs to pass: side-stepping
/// detours leave the mesh, so near-wall avoidance must reject them and fall
/// back to contained candidates (deadlock is acceptable per SIM-012).
[[nodiscard]] NavMesh corridor_mesh()
{
    return make_mesh({Tri(V(0.0f, 0.0f), V(10.0f, 0.0f), V(10.0f, 0.5f)),
                      Tri(V(0.0f, 0.0f), V(10.0f, 0.5f), V(0.0f, 0.5f))});
}

[[nodiscard]] AgentConfig cfg(Vec2 position, float speed, Vec2 goal) noexcept
{
    return AgentConfig{position, kRadius, speed, goal, -1.0f};
}

struct RunSummary {
    bool ok{true};
    std::string failure;
    std::size_t failure_step{0};
    std::vector<Vec2> trace_a;
    std::vector<Vec2> trace_b;
    float min_separation{std::numeric_limits<float>::infinity()};
    bool both_reached{false};
    std::size_t steps_taken{0};
};

/// Steps two agents until both reach their goals or the budget runs out,
/// checking the per-substep invariants: finite states, mesh containment,
/// speed bound, and — once separation has been latched — no overlap.
RunSummary run_pair(Simulation& sim, AgentId a, AgentId b, const NavMesh& mesh,
                    std::size_t max_steps = kMaxSteps)
{
    RunSummary out;
    const auto initial_a = sim.agent(a);
    const auto initial_b = sim.agent(b);
    if (initial_a == std::nullopt || initial_b == std::nullopt) {
        out.ok = false;
        out.failure = "agent missing before the run";
        return out;
    }
    bool latched = distance(initial_a->position, initial_b->position) >= kLatch;

    for (std::size_t i = 0; i < max_steps; ++i) {
        if (!sim.step(kDt).has_value()) {
            out.ok = false;
            out.failure = "step failed";
            out.failure_step = i;
            break;
        }
        const auto sa = sim.agent(a);
        const auto sb = sim.agent(b);
        if (sa == std::nullopt || sb == std::nullopt) {
            out.ok = false;
            out.failure = "agent vanished mid-run";
            out.failure_step = i;
            break;
        }
        out.trace_a.push_back(sa->position);
        out.trace_b.push_back(sb->position);
        for (const AgentState* s : {&*sa, &*sb}) {
            if (!std::isfinite(s->position.x) || !std::isfinite(s->position.y) ||
                !std::isfinite(s->velocity.x) || !std::isfinite(s->velocity.y)) {
                out.ok = false;
                out.failure = "non-finite state";
                out.failure_step = i;
                break;
            }
            if (!mesh.contains(s->position)) {
                out.ok = false;
                out.failure = "position left the mesh";
                out.failure_step = i;
                break;
            }
            if (length(s->velocity) > s->max_speed * (1.0f + 1e-5f)) {
                out.ok = false;
                out.failure = "velocity exceeded max_speed";
                out.failure_step = i;
                break;
            }
        }
        if (!out.ok) {
            break;
        }
        const float sep = distance(sa->position, sb->position);
        if (!latched && sep >= kLatch) {
            latched = true;
        }
        if (latched) {
            out.min_separation = std::min(out.min_separation, sep);
            if (sep < kEnforce) {
                out.ok = false;
                out.failure = "agents overlapped after separating";
                out.failure_step = i;
                break;
            }
        }
        out.steps_taken = i + 1;
        if (sa->status == AgentStatus::Reached && sb->status == AgentStatus::Reached) {
            out.both_reached = true;
            break;
        }
    }
    return out;
}

/// Records one agent's position each substep until it reaches its goal or the
/// step budget runs out.
std::vector<Vec2> trace_until_reached(Simulation& sim, AgentId a)
{
    std::vector<Vec2> trace;
    for (std::size_t i = 0; i < kMaxSteps; ++i) {
        if (!sim.step(kDt).has_value()) {
            ADD_FAILURE() << "step failed at " << i;
            break;
        }
        const auto s = sim.agent(a);
        if (!s.has_value()) {
            ADD_FAILURE() << "agent vanished at " << i;
            break;
        }
        trace.push_back(s->position);
        if (s->status == AgentStatus::Reached) {
            break;
        }
    }
    return trace;
}

TEST(SimulationAvoidance, CrossingPathsStaySeparatedAndBothReachGoals)
{
    const NavMesh mesh = open_mesh();
    Simulation sim(mesh);
    // Both agents arrive at the intersection (10, 4) at t = 2 s: a genuine
    // collision course that forces one of them to yield.
    const AgentId a = add_or_fail(sim, cfg(V(7.0f, 4.0f), 1.5f, V(16.0f, 4.0f)));
    const AgentId b = add_or_fail(sim, cfg(V(10.0f, 2.0f), 1.0f, V(10.0f, 12.0f)));

    const RunSummary run = run_pair(sim, a, b, mesh);
    EXPECT_TRUE(run.ok) << run.failure << " at step " << run.failure_step;
    EXPECT_TRUE(run.both_reached) << "steps taken: " << run.steps_taken;
    EXPECT_GE(run.min_separation, kEnforce);

    const auto sa = sim.agent(a);
    const auto sb = sim.agent(b);
    ASSERT_TRUE(sa.has_value() && sb.has_value());
    EXPECT_LE(distance(sa->position, V(16.0f, 4.0f)), kRadius + 1e-3f);
    EXPECT_LE(distance(sb->position, V(10.0f, 12.0f)), kRadius + 1e-3f);
}

TEST(SimulationAvoidance, OvertakingAgentPassesWithoutOverlap)
{
    const NavMesh mesh = open_mesh();
    Simulation sim(mesh);
    // Same northbound corridor: the faster follower starts 2 m behind and
    // must slow down or swing around the leader instead of pushing through.
    const AgentId leader = add_or_fail(sim, cfg(V(10.0f, 3.0f), 1.0f, V(10.0f, 13.0f)));
    const AgentId follower = add_or_fail(sim, cfg(V(10.0f, 1.0f), 2.0f, V(10.0f, 11.0f)));

    const RunSummary run = run_pair(sim, follower, leader, mesh);
    EXPECT_TRUE(run.ok) << run.failure << " at step " << run.failure_step;
    EXPECT_TRUE(run.both_reached) << "steps taken: " << run.steps_taken;
    EXPECT_GE(run.min_separation, kEnforce);
}

TEST(SimulationAvoidance, HeadOnInNarrowCorridorStaysContainedAndSeparated)
{
    const NavMesh mesh = corridor_mesh();
    Simulation sim(mesh);
    // Head-on along the corridor centreline; neither can slip past the other.
    const AgentId a = add_or_fail(sim, cfg(V(1.0f, 0.25f), 1.0f, V(9.0f, 0.25f)));
    const AgentId b = add_or_fail(sim, cfg(V(9.0f, 0.25f), 1.0f, V(1.0f, 0.25f)));

    // Run the full budget: they may deadlock, but must never leave the mesh,
    // blow up, exceed max_speed, or overlap beyond tolerance (run_pair checks
    // all of these per substep once separation has latched, which it is here).
    const RunSummary run = run_pair(sim, a, b, mesh);
    EXPECT_TRUE(run.ok) << run.failure << " at step " << run.failure_step;
    EXPECT_GE(run.min_separation, kEnforce);
    // Not asserting both_reached: a head-on deadlock in a corridor is allowed.
}

TEST(SimulationAvoidance, InitiallyOverlappingAgentsSeparateThenReachGoals)
{
    const NavMesh mesh = open_mesh();
    Simulation sim(mesh);
    ASSERT_LT(distance(V(5.0f, 5.0f), V(5.0f, 5.4f)), kRadiusSum);
    const AgentId a = add_or_fail(sim, cfg(V(5.0f, 5.0f), 1.0f, V(2.0f, 2.0f)));
    const AgentId b = add_or_fail(sim, cfg(V(5.0f, 5.4f), 1.0f, V(15.0f, 2.0f)));

    const RunSummary run = run_pair(sim, a, b, mesh);
    EXPECT_TRUE(run.ok) << run.failure << " at step " << run.failure_step;
    EXPECT_TRUE(run.both_reached) << "steps taken: " << run.steps_taken;
    // min_separation is tracked after the latch, so this asserts the pair
    // separated and then stayed separated all the way to the goals.
    EXPECT_GE(run.min_separation, kEnforce);
}

TEST(SimulationAvoidance, CrossingReplayIsBitwiseIdentical)
{
    const auto play = [] {
        const NavMesh mesh = open_mesh();
        Simulation sim(mesh);
        const AgentId a = add_or_fail(sim, cfg(V(7.0f, 4.0f), 1.5f, V(16.0f, 4.0f)));
        const AgentId b = add_or_fail(sim, cfg(V(10.0f, 2.0f), 1.0f, V(10.0f, 12.0f)));
        return run_pair(sim, a, b, mesh);
    };
    const RunSummary first = play();
    const RunSummary second = play();
    ASSERT_TRUE(first.ok && second.ok);
    ASSERT_TRUE(first.both_reached && second.both_reached);
    EXPECT_EQ(first.trace_a, second.trace_a);
    EXPECT_EQ(first.trace_b, second.trace_b);
    EXPECT_EQ(first.steps_taken, second.steps_taken);
}

TEST(SimulationAvoidance, DistantAgentsDoNotInfluenceEachOther)
{
    const NavMesh mesh = open_mesh();

    // Solo run of agent A.
    std::vector<Vec2> solo;
    {
        Simulation sim(mesh);
        const AgentId a = add_or_fail(sim, cfg(V(7.0f, 4.0f), 1.5f, V(16.0f, 4.0f)));
        solo = trace_until_reached(sim, a);
    }

    // Same agent with a second agent far outside its interaction range.
    std::vector<Vec2> paired;
    {
        Simulation sim(mesh);
        const AgentId a = add_or_fail(sim, cfg(V(7.0f, 4.0f), 1.5f, V(16.0f, 4.0f)));
        // A second agent far outside A's interaction range; its id is never
        // consulted — only its presence in the simulation matters here.
        add_or_fail(sim, cfg(V(1.0f, 1.0f), 0.5f, V(2.0f, 1.0f)));
        ASSERT_GT(distance(V(7.0f, 4.0f), V(1.0f, 1.0f)), 6.0f);
        paired = trace_until_reached(sim, a);
    }

    EXPECT_EQ(solo, paired);
}

// A concave L-shaped mesh: bar [0,2]x[0,1] + arm [0,1]x[1,3], reflex corner
// at (1,1), out-of-mesh notch for x>1 && y>1. Two agents converge on the
// corner and must detour around each other. SIM-012 requires every substep
// position to stay in the mesh; this guards against detours that tunnel
// across the notch and off-route walk-backs that cut the concave corner.
// Separation is not asserted: the arm is barely wider than two discs, so
// near-wall deadlock (allowed by SIM-012) would make a separation check
// geometry-dependent rather than containment-focused.
TEST(SimulationAvoidance, ConcaveCornerAvoidanceStaysContained)
{
    const NavMesh mesh = make_l_mesh();
    Simulation sim(mesh);
    const AgentId a = add_or_fail(sim, cfg(V(1.7f, 0.4f), 1.0f, V(0.4f, 2.6f)));
    const AgentId b = add_or_fail(sim, cfg(V(0.4f, 2.0f), 1.0f, V(0.4f, 0.4f)));
    for (std::size_t step = 0; step < kMaxSteps; ++step) {
        ASSERT_TRUE(sim.step(kDt).has_value()) << "step failed at " << step;
        for (const AgentId id : {a, b}) {
            const auto s = sim.agent(id);
            ASSERT_TRUE(s.has_value()) << "agent missing at step " << step;
            ASSERT_TRUE(mesh.contains(s->position))
                << "agent left the mesh at step " << step << " position (" << s->position.x << ", "
                << s->position.y << ")";
        }
    }
}

} // namespace
