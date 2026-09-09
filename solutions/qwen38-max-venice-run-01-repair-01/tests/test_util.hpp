#pragma once

// Shared helpers for the vwmini test suite.

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <utility>
#include <vector>

namespace vwmini::test {

/** Builds a Vec2; a function call avoids braced-initializer commas confusing the GTest macros. */
inline constexpr Vec2 V(float x, float y) noexcept
{
    return Vec2{x, y};
}

/** Builds a Polygon from its vertex list. */
inline Polygon Poly(std::vector<Vec2> pts)
{
    return Polygon{std::move(pts)};
}

/** Builds a single-triangle Polygon from three points. */
inline Polygon Tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{std::vector<Vec2>{a, b, c}};
}

/** Extracts the error code of a failed `Result`; only valid on errors. */
template <class T> [[nodiscard]] ErrorCode code_of(const Result<T>& r)
{
    return r.error().code;
}

/** Creates a NavMesh from triangles; ADD_FAILURE on error. The value() call
 * throws only after the test has already failed, so callers never see an
 * invalid mesh. */
inline NavMesh make_mesh(std::vector<Polygon> triangles)
{
    auto mesh = NavMesh::create(std::move(triangles));
    if (!mesh.has_value()) {
        ADD_FAILURE() << "NavMesh::create failed: " << mesh.error().message;
    }
    return std::move(mesh).value();
}

/** Two CCW triangles tiling the square [lo, lo+size]^2. */
inline std::vector<Polygon> square_triangles(Vec2 lo, float size)
{
    const Vec2 hi{lo.x + size, lo.y + size};
    return {Tri(lo, Vec2{hi.x, lo.y}, hi), Tri(lo, hi, Vec2{lo.x, hi.y})};
}

/** Triangulates a simple outline; returns {} (and fails the test) on error. */
inline std::vector<Polygon> outline_triangles(std::vector<Vec2> outline)
{
    auto triangles = triangulate_simple_polygon(Polygon{std::move(outline)});
    if (!triangles.has_value()) {
        ADD_FAILURE() << "triangulate failed: " << triangles.error().message;
        return {};
    }
    return std::move(*triangles);
}

/** L outline (CCW): horizontal bar [0,2]x[0,1] plus vertical arm [0,1]x[1,3];
 * reflex corner at (1,1). */
inline std::vector<Vec2> l_outline()
{
    return {V(0, 0), V(2, 0), V(2, 1), V(1, 1), V(1, 3), V(0, 3)};
}

/** Builds the L mesh from `l_outline()` through the full public pipeline. */
inline NavMesh make_l_mesh()
{
    return make_mesh(outline_triangles(l_outline()));
}

/** Adds one agent; ADD_FAILURE on error. The value() call throws only after
 * the test has already failed, so callers never see an invented id. */
inline AgentId add_or_fail(Simulation& sim, const AgentConfig& config)
{
    auto id = sim.add_agent(config);
    if (!id.has_value()) {
        ADD_FAILURE() << "add_agent failed: " << id.error().message;
    }
    return std::move(id).value();
}

/** Returns a live agent snapshot; fails the test when the id is unknown. */
inline AgentState state_or_fail(const Simulation& sim, AgentId id)
{
    const std::optional<AgentState> s = sim.agent(id);
    EXPECT_TRUE(s.has_value());
    return s.value_or(AgentState{});
}

} // namespace vwmini::test
