#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include "mesh_fixtures.hpp"

#include <gtest/gtest.h>

#include <cfloat>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

using vwmini::AgentConfig;
using vwmini::AgentStatus;
using vwmini::NavMesh;
using vwmini::Simulation;
using vwmini::Vec2;

/// A huge but finite counter-clockwise triangle: the products of its edge components overflow
/// `float`, which is exactly what the internal numeric policy has to survive.
[[nodiscard]] NavMesh huge_triangle_mesh()
{
    return fixtures::make_mesh(std::vector<fixtures::Polygon>{
        fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{3.0e38f, 0.0f}, Vec2{0.0f, 3.0e38f})});
}

TEST(NumericsTest, LengthStaysFiniteForExtremeFiniteVectors)
{
    const float huge = std::numeric_limits<float>::max();
    // The true norm of (FLT_MAX, FLT_MAX) is not representable as a float, so it saturates at
    // the largest finite float instead of returning infinity.
    EXPECT_EQ(vwmini::length(Vec2{huge, huge}), huge);
    EXPECT_TRUE(std::isfinite(vwmini::length(Vec2{huge, 0.0f})));

    // Tiny finite vectors keep a finite, non-zero length instead of under-flowing to zero.
    const float tiny = std::numeric_limits<float>::denorm_min();
    EXPECT_GT(vwmini::length(Vec2{tiny, tiny}), 0.0f);

    // Ordinary magnitudes are unchanged.
    EXPECT_NEAR(vwmini::length(Vec2{3.0f, 4.0f}), 5.0f, 1.0e-6f);

    // Non-finite input propagates rather than being silently clamped.
    EXPECT_TRUE(std::isinf(vwmini::length(Vec2{std::numeric_limits<float>::infinity(), 1.0f})));
}

TEST(NumericsTest, NormalizedKeepsDirectionForExtremeAndTinyVectors)
{
    const float huge = std::numeric_limits<float>::max();
    const Vec2 extreme = vwmini::normalized(Vec2{huge, huge});
    ASSERT_TRUE(std::isfinite(extreme.x) && std::isfinite(extreme.y));
    EXPECT_NEAR(vwmini::length(extreme), 1.0f, 1.0e-5f);
    EXPECT_GT(extreme.x, 0.0f);
    EXPECT_GT(extreme.y, 0.0f);

    const Vec2 asymmetric = vwmini::normalized(
        Vec2{-std::numeric_limits<float>::max(), std::numeric_limits<float>::max() / 2.0f});
    ASSERT_TRUE(std::isfinite(asymmetric.x) && std::isfinite(asymmetric.y));
    EXPECT_LT(asymmetric.x, 0.0f);
    EXPECT_GT(asymmetric.y, 0.0f);
    EXPECT_NEAR(vwmini::length(asymmetric), 1.0f, 1.0e-5f);

    const float tiny = std::numeric_limits<float>::denorm_min();
    const Vec2 small = vwmini::normalized(Vec2{tiny, 0.0f});
    ASSERT_TRUE(std::isfinite(small.x) && std::isfinite(small.y));
    EXPECT_NEAR(vwmini::length(small), 1.0f, 1.0e-3f);

    EXPECT_EQ(vwmini::normalized(Vec2{0.0f, 0.0f}), Vec2(0.0f, 0.0f));
}

TEST(NumericsTest, ExtremeTriangleContainsItsInteriorPoint)
{
    const NavMesh mesh = huge_triangle_mesh();
    EXPECT_EQ(mesh.cell_count(), 1u);

    // Strictly interior: the point satisfies x + y < 3e38, but every cross product of the
    // edge vectors overflows float.
    EXPECT_TRUE(mesh.contains(Vec2{1.0e38f, 1.0e38f}));

    // The edges and the far side of them stay distinguishable.
    EXPECT_TRUE(mesh.contains(Vec2{1.5e38f, 0.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{2.5e38f, 1.5e38f}));
    EXPECT_FALSE(mesh.contains(Vec2{-1.0e38f, 1.0e38f}));
}

TEST(NumericsTest, ExtremeTriangleSupportsPathQueries)
{
    const NavMesh mesh = huge_triangle_mesh();
    const auto path = vwmini::find_path(mesh, Vec2{1.0e38f, 1.0e38f}, Vec2{0.5e38f, 0.4e38f});
    ASSERT_TRUE(path.has_value()) << path.error().message;
    ASSERT_EQ(path->points.size(), 2u);
    EXPECT_EQ(path->points.front(), Vec2(1.0e38f, 1.0e38f));
    EXPECT_EQ(path->points.back(), Vec2(0.5e38f, 0.4e38f));

    const auto outside = vwmini::find_path(mesh, Vec2{1.0e38f, 1.0e38f}, Vec2{3.0e38f, 3.0e38f});
    ASSERT_FALSE(outside.has_value());
    EXPECT_EQ(outside.error().code, vwmini::ErrorCode::OutsideMesh);
}

/// The L corridor of the ordinary tests, translated so far from the origin that one float ULP
/// is centimetres: the geometry stays usable and the route still has to turn the corner.
TEST(NumericsTest, RoutesOnAMeshFarFromTheOrigin)
{
    const NavMesh mesh =
        fixtures::make_mesh(fixtures::offset(fixtures::l_corridor(), Vec2{1.0e6f, 0.0f}));

    const Vec2 start{1.0e6f + 0.3f, 0.5f};
    const Vec2 goal{1.0e6f + 1.5f, 2.7f};
    ASSERT_TRUE(mesh.contains(start));
    ASSERT_TRUE(mesh.contains(goal));

    const auto path = vwmini::find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value()) << path.error().message;
    EXPECT_GT(path->points.size(), 2u) << "the route must still turn the corner";
    EXPECT_EQ(path->points.front(), start);
    EXPECT_EQ(path->points.back(), goal);
    for (const Vec2 &point : path->points)
    {
        EXPECT_TRUE(mesh.contains(point));
    }
}

TEST(NumericsTest, AgentsMoveOnAMeshFarFromTheOrigin)
{
    const NavMesh mesh =
        fixtures::make_mesh(fixtures::offset(fixtures::square_pair(10.0f), Vec2{1.0e4f, 0.0f}));
    Simulation simulation{NavMesh{mesh}};
    const Vec2 start{1.0e4f + 2.0f, 5.0f};
    const Vec2 goal{1.0e4f + 8.0f, 5.0f};
    const auto id = simulation.add_agent(AgentConfig{start, 0.25f, 1.0f, goal, 0.1f}).value();

    for (int index = 0; index < 400; ++index)
    {
        ASSERT_TRUE(simulation.step(0.05f).has_value());
        const auto state = simulation.agent(id);
        ASSERT_TRUE(state.has_value());
        EXPECT_TRUE(std::isfinite(state->position.x) && std::isfinite(state->position.y))
            << "agent position went non-finite far from the origin";
    }
    ASSERT_TRUE(simulation.agent(id).has_value());
    EXPECT_EQ(simulation.agent(id)->status, AgentStatus::Reached);
    EXPECT_NEAR(vwmini::length(*simulation.agent(id)->goal - simulation.agent(id)->position), 0.1f,
                1.0e-2f);
}

/**
 * A convex strip two float exponents wide.
 *
 * Its width, 6e38, exceeds `FLT_MAX`: arithmetic that subtracts the extreme x coordinates in
 * `float` overflows even though every coordinate involved is an accepted finite value.
 */
[[nodiscard]] NavMesh half_float_range_strip()
{
    return fixtures::make_mesh(std::vector<fixtures::Polygon>{
        fixtures::triangle(Vec2{-3.0e38f, 0.0f}, Vec2{3.0e38f, 0.0f}, Vec2{3.0e38f, 1.0e37f}),
        fixtures::triangle(Vec2{-3.0e38f, 0.0f}, Vec2{3.0e38f, 1.0e37f}, Vec2{-3.0e38f, 1.0e37f})});
}

TEST(NumericsTest, RouteAcrossAStripWiderThanTheFloatRangeIsStraight)
{
    const NavMesh mesh = half_float_range_strip();
    const Vec2 start{-2.0e38f, 0.9e37f};
    const Vec2 goal{2.0e38f, 1.0e36f};
    ASSERT_TRUE(mesh.contains(start));
    ASSERT_TRUE(mesh.contains(goal));

    // The straight segment lies inside the strip, so the route must be exactly its two endpoints
    // (SIM-001). An overflowed delta reported a detour through the portal vertices instead.
    const auto path = vwmini::find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value()) << path.error().message;
    EXPECT_EQ(path->points.size(), 2u);
}

TEST(NumericsTest, AgentSteersAcrossAnOffsetWiderThanTheFloatRange)
{
    const Vec2 start{-2.0e38f, 5.0e36f};
    const Vec2 goal{2.0e38f, 5.0e36f};
    Simulation simulation{half_float_range_strip()};
    // Fast enough that the progress of one *substep* is larger than one float ULP at 2e38
    // (~2e31). A position is a `float`: what it cannot represent does not happen, so an agent
    // slower than this keeps its coordinate and no amount of stepping moves it (see
    // `SmallStepsFarFromTheOriginRespectTheSpeedBudget`).
    const auto id = simulation.add_agent(AgentConfig{start, 1.0f, 1.0e34f, goal}).value();

    for (int index = 0; index < 5; ++index)
    {
        ASSERT_TRUE(simulation.step(1.0f).has_value());
    }
    const auto state = simulation.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_TRUE(std::isfinite(state->position.x) && std::isfinite(state->position.y));
    // Before the repair the goal offset overflowed to infinity, normalization discarded it and
    // the agent kept a zero velocity while still reported as Moving.
    EXPECT_GT(state->position.x, start.x);
    EXPECT_GT(state->velocity.x, 0.0f);
    EXPECT_LE(vwmini::length(state->velocity), 1.0e34f);
}

TEST(NumericsTest, StoredVelocityDoesNotExceedTheConfiguredMaximumSpeed)
{
    const NavMesh mesh = fixtures::make_mesh(fixtures::square_pair(10.0f));
    Simulation simulation{NavMesh{mesh}};
    const auto id =
        simulation.add_agent(AgentConfig{Vec2{1.0f, 5.0f}, 0.25f, 1.4f, Vec2{9.0f, 5.0f}}).value();
    ASSERT_TRUE(simulation.step(0.1f).has_value());
    const auto state = simulation.agent(id);
    ASSERT_TRUE(state.has_value());
    // The limit is a cap, not an approximate target: narrowing the double velocity to float must
    // not round the stored magnitude above it.
    EXPECT_LE(vwmini::length(state->velocity), 1.4f);
}

/**
 * SIM-008 far from the origin: movement may never exceed the speed budget.
 *
 * One float ULP near x=1e4 is 9.8e-4, so a 1 m/s agent stepping 1e-4 s cannot advance at all and
 * `Vec2` simply stays put. An integrator that banked the unrepresentable remainder and released
 * it later made the fifth call jump by a whole ULP (0.0009765625) although only 0.0005 s of
 * travel had been paid for; the per-call bound below catches exactly that.
 */
TEST(NumericsTest, SmallStepsFarFromTheOriginRespectTheSpeedBudget)
{
    const NavMesh mesh =
        fixtures::make_mesh(fixtures::offset(fixtures::square_pair(10.0f), Vec2{1.0e4f, 0.0f}));
    Simulation simulation{NavMesh{mesh}};
    constexpr float kStep = 1.0e-4f;
    constexpr float kMaxSpeed = 1.0f;
    const Vec2 start{1.0e4f + 1.0f, 5.0f};
    const auto id =
        simulation.add_agent(AgentConfig{start, 0.25f, kMaxSpeed, Vec2{1.0e4f + 8.0f, 5.0f}})
            .value();

    Vec2 previous = start;
    double elapsed = 0.0;
    double travelled = 0.0;
    for (int index = 0; index < 5; ++index)
    {
        ASSERT_TRUE(simulation.step(kStep).has_value());
        elapsed += double(kStep);
        const auto state = simulation.agent(id);
        ASSERT_TRUE(state.has_value());
        const double dx = double(state->position.x) - double(previous.x);
        const double dy = double(state->position.y) - double(previous.y);
        const double moved = std::sqrt(dx * dx + dy * dy);
        travelled += moved;
        EXPECT_LE(moved, double(kMaxSpeed) * double(kStep))
            << "call " << index << " moved further than its own slice of time allows";
        EXPECT_LE(travelled, double(kMaxSpeed) * elapsed)
            << "cumulative movement passed the elapsed allowance";
        EXPECT_TRUE(std::isfinite(state->position.x) && std::isfinite(state->position.y));
        previous = state->position;
    }
}

/**
 * A triangle whose right edge is `FLT_MAX` itself.
 *
 * Motion near that edge has to decide containment for a candidate whose double value is already
 * past what a `float` can hold: narrowing that checks its range first answers "no", while an
 * unchecked cast hands out infinities.
 */
[[nodiscard]] NavMesh float_limit_triangle()
{
    return fixtures::make_mesh(std::vector<fixtures::Polygon>{fixtures::triangle(
        Vec2{3.0e38f, -1.0e37f}, Vec2{FLT_MAX, -1.0e37f}, Vec2{FLT_MAX, 1.0e37f})});
}

/**
 * A step bigger than the float range, deflected away from the goal (SIM-008/SIM-012).
 *
 * The peer sits on top of the agent to its west, so separation drives it east at its full
 * 1.5e38 m/s while its goal lies west: every substep wants 1.4e36 m more than `3.39e38` and the
 * double candidate therefore passes `FLT_MAX`. State must stay finite, contained and inside the
 * speed cap, and the mesh edge must never be crossed.
 */
TEST(NumericsTest, ExtremeDeflectionPastTheFloatLimitStaysFiniteContainedAndCapped)
{
    const NavMesh mesh = float_limit_triangle();
    Simulation simulation{float_limit_triangle()};
    constexpr Vec2 kStart{3.39e38f, 0.0f};
    constexpr Vec2 kGoal{3.25e38f, 0.0f};
    ASSERT_TRUE(mesh.contains(kStart));
    ASSERT_TRUE(mesh.contains(kGoal));

    const auto blocker =
        simulation.add_agent(AgentConfig{Vec2{3.3885e38f, 0.0f}, 1.0e36f, 1.0f}).value();
    const auto id = simulation.add_agent(AgentConfig{kStart, 1.0e36f, 1.5e38f, kGoal}).value();

    constexpr float kDuration = 0.5f;
    ASSERT_TRUE(simulation.step(kDuration).has_value());
    const auto state = simulation.agent(id);
    ASSERT_TRUE(state.has_value());
    EXPECT_TRUE(std::isfinite(state->position.x) && std::isfinite(state->position.y))
        << "an unrepresentable position was stored: " << state->position.x << ", "
        << state->position.y;
    EXPECT_TRUE(mesh.contains(state->position)) << "motion left the mesh";
    EXPECT_LE(state->position.x, FLT_MAX);
    const double dx = double(state->position.x) - double(kStart.x);
    const double dy = double(state->position.y);
    EXPECT_LE(std::sqrt(dx * dx + dy * dy), 1.5e38 * 0.5) << "moved past the speed budget";
    EXPECT_TRUE(std::isfinite(state->velocity.x) && std::isfinite(state->velocity.y));
    EXPECT_LE(vwmini::length(state->velocity), 1.5e38f);

    // The blocked peer has no goal and must not have drifted.
    const auto still = simulation.agent(blocker);
    ASSERT_TRUE(still.has_value());
    const Vec2 parked{3.3885e38f, 0.0f};
    EXPECT_EQ(still->position, parked);

    // And the simulation is still usable at the edge of the range.
    ASSERT_TRUE(simulation.step(kDuration).has_value());
    EXPECT_TRUE(std::isfinite(simulation.agent(id)->position.x));
    EXPECT_TRUE(mesh.contains(simulation.agent(id)->position));
}

} // namespace