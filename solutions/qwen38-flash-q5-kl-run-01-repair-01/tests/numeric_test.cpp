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
    // Fast enough that one second of travel is larger than one float ULP at 2e38 (~2e31): below
    // that, the stored position cannot represent the progress even though the velocity is right.
    const auto id = simulation.add_agent(AgentConfig{start, 1.0f, 1.0e33f, goal}).value();

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
    EXPECT_LE(vwmini::length(state->velocity), 1.0e33f);
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

} // namespace
