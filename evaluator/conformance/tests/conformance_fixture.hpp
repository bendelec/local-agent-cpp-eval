#pragma once

// Shared helpers for the published black-box conformance suite.
//
// These helpers use ONLY the public API (include/vwmini/*) and hand-authored literal
// triangles. They never call the candidate triangulator and never reach into
// implementation internals, so they link against any implementation exposing the
// canonical vwmini::vwmini target.
//
// CMake selection (see evaluator/README.md):
//   - The conformance executable links the target `vwmini::vwmini`.
//   - `VWMINI_CONFORMANCE_SOURCE_DIR` selects the provider:
//       * unset/empty  -> evaluator/reference (add_subdirectory(../reference))
//       * set          -> external candidate project (add_subdirectory(<dir>)),
//                         which must itself define `vwmini::vwmini`.
//   - The GTest dependency is resolved via find_package(GTest CONFIG REQUIRED).

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace vwmini_conformance {

inline constexpr float kEps = 1e-4f;

/// Component-wise "close enough" comparison under the geometric tolerance.
[[nodiscard]] inline bool near(vwmini::Vec2 a, vwmini::Vec2 b, float eps = kEps) noexcept
{
    return std::fabs(a.x - b.x) <= eps && std::fabs(a.y - b.y) <= eps;
}

[[nodiscard]] inline bool is_finite(vwmini::Vec2 v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y);
}

/// Signed area (shoelace) of a polygon, in double, positive for CCW.
[[nodiscard]] inline double signed_area(const vwmini::Polygon& polygon) noexcept
{
    double sum = 0.0;
    const std::size_t n = polygon.vertices.size();
    for (std::size_t i = 0; i < n; ++i) {
        const vwmini::Vec2 p = polygon.vertices[i];
        const vwmini::Vec2 q = polygon.vertices[(i + 1u) % n];
        sum += static_cast<double>(p.x) * q.y - static_cast<double>(p.y) * q.x;
    }
    return sum * 0.5;
}

/// Asserts a Result carries the expected error code (and no value).
template <class T>
void expect_error(const vwmini::Result<T>& result, vwmini::ErrorCode code)
{
    ASSERT_FALSE(result.has_value()) << "expected an error result";
    EXPECT_EQ(result.error().code, code);
}

/// True when the closed segment [a, b] is contained by the mesh under dense sampling.
/// Sampling is adequate for the well-separated fixtures used by this suite.
[[nodiscard]] inline bool segment_contained(const vwmini::NavMesh& mesh, vwmini::Vec2 a,
                                            vwmini::Vec2 b, int samples = 128) noexcept
{
    for (int i = 0; i <= samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples);
        if (!mesh.contains(a + (b - a) * t)) {
            return false;
        }
    }
    return true;
}

/// True when a path satisfies SIM-002's structural contract: non-empty, preserves the
/// caller's start and goal, every segment is contained, and consecutive points are no
/// closer than epsilon.
[[nodiscard]] inline bool path_is_valid(const vwmini::NavMesh& mesh, const vwmini::Path& path,
                                        vwmini::Vec2 start, vwmini::Vec2 goal) noexcept
{
    if (path.points.empty()) {
        return false;
    }
    if (!(path.points.front() == start) || !(path.points.back() == goal)) {
        return false;
    }
    for (std::size_t i = 0; i + 1u < path.points.size(); ++i) {
        if (near(path.points[i], path.points[i + 1u])) {
            return false;
        }
        if (!segment_contained(mesh, path.points[i], path.points[i + 1u])) {
            return false;
        }
    }
    return true;
}

// ---- Hand-authored literal meshes (never produced by the triangulator). ----

/// The 10x10 CCW square split into the two CCW triangles used by the crowd scenarios.
[[nodiscard]] inline vwmini::NavMesh make_square_mesh()
{
    auto result = vwmini::NavMesh::create({
        vwmini::Polygon{{{0.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, 10.0f}}},
        vwmini::Polygon{{{0.0f, 0.0f}, {10.0f, 10.0f}, {0.0f, 10.0f}}},
    });
    return std::move(result).value();
}

/// A single CCW right triangle.
[[nodiscard]] inline vwmini::NavMesh make_single_triangle_mesh()
{
    auto result = vwmini::NavMesh::create({
        vwmini::Polygon{{{0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}}},
    });
    return std::move(result).value();
}

/// An L-shaped mesh: a 2x2 grid of unit cells with the top-right cell removed. Six CCW
/// unit triangles. The reflex corner is (1,1); a straight line from (1.9,0.5) to
/// (0.5,1.9) cuts through the missing cell, forcing a bent route.
[[nodiscard]] inline vwmini::NavMesh make_l_mesh()
{
    auto result = vwmini::NavMesh::create({
        // bottom-left cell [0,1]x[0,1]
        vwmini::Polygon{{{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}}},
        vwmini::Polygon{{{0.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}}},
        // bottom-right cell [1,2]x[0,1]
        vwmini::Polygon{{{1.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f}}},
        vwmini::Polygon{{{1.0f, 0.0f}, {2.0f, 1.0f}, {1.0f, 1.0f}}},
        // top-left cell [0,1]x[1,2]
        vwmini::Polygon{{{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 2.0f}}},
        vwmini::Polygon{{{0.0f, 1.0f}, {1.0f, 2.0f}, {0.0f, 2.0f}}},
    });
    return std::move(result).value();
}

/// Two disjoint CCW triangles: a valid mesh whose components are not connected.
[[nodiscard]] inline vwmini::NavMesh make_disconnected_mesh()
{
    auto result = vwmini::NavMesh::create({
        vwmini::Polygon{{{0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}}},
        vwmini::Polygon{{{20.0f, 0.0f}, {30.0f, 0.0f}, {20.0f, 10.0f}}},
    });
    return std::move(result).value();
}

/// Three disconnected triangles whose bases lie within MSH-006's epsilon band of y=0.
/// The points (0,0), (5,0), and (10,0) are individually contained, but the intervals
/// between them are uncovered and must not become a traversable connection.
[[nodiscard]] inline vwmini::NavMesh make_epsilon_band_islands_mesh()
{
    constexpr float base_y = kEps / 2.0f;
    auto result = vwmini::NavMesh::create({
        vwmini::Polygon{{{-1.0f, base_y}, {1.0f, base_y}, {0.0f, 150.0f}}},
        vwmini::Polygon{{{4.0f, base_y}, {6.0f, base_y}, {5.0f, 1.0f}}},
        vwmini::Polygon{{{9.0f, base_y}, {11.0f, base_y}, {10.0f, 1.0f}}},
    });
    return std::move(result).value();
}

} // namespace vwmini_conformance
