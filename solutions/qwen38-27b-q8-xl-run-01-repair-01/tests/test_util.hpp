// Shared helpers for VWmini tests.
#pragma once

#include <vwmini/geometry.hpp>

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

namespace vwmini::testing {

constexpr float kEps = 1e-4f;

// Precondition: `result` is disengaged. Exposes the error code of a failed
// `Result` for EXPECT_EQ assertions.
[[nodiscard]] inline ErrorCode err_code(const auto& result) noexcept
{
    return result.error().code;
}

// Triangle with the given vertices, CCW by construction when listed CCW.
[[nodiscard]] inline Polygon tri(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return Polygon{{a, b, c}};
}

// Sum of signed areas of a triangle list (float).
[[nodiscard]] inline double signed_area_sum(const std::vector<Polygon>& polys) noexcept
{
    double twice = 0.0;
    for (const Polygon& p : polys)
        for (std::size_t i = 0; i < p.vertices.size(); ++i)
        {
            const auto& a = p.vertices[i];
            const auto& b = p.vertices[(i + 1) % p.vertices.size()];
            twice += static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
        }
    return twice * 0.5;
}

// Strict orientation sign (0, +1, -1) for point p against directed segment ab.
[[nodiscard]] inline int orient_sign(Vec2 a, Vec2 b, Vec2 p) noexcept
{
    const float c = cross(b - a, p - a);
    return (c > 0.0f) - (c < 0.0f);
}

// Two closed triangles have disjoint interiors (test-side mesh validity).
[[nodiscard]] inline bool triangle_interiors_disjoint(const Polygon& t1,
                                                      const Polygon& t2) noexcept
{
    const auto& a = t1.vertices;
    const auto& b = t2.vertices;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
        {
            const Vec2& b0 = b[j];
            const Vec2& b1 = b[(j + 1) % 3];
            const Vec2& a0 = a[i];
            const Vec2& a1 = a[(i + 1) % 3];
            const float o1 = cross(b1 - b0, a0 - b0);
            const float o2 = cross(b1 - b0, a1 - b0);
            const float o3 = cross(a1 - a0, b0 - a0);
            const float o4 = cross(a1 - a0, b1 - a0);
            const bool straddle =
                ((o1 > 0.0f && o2 < 0.0f) || (o1 < 0.0f && o2 > 0.0f)) &&
                ((o3 > 0.0f && o4 < 0.0f) || (o3 < 0.0f && o4 > 0.0f));
            if (straddle)
                return false;
        }
    for (const Vec2& p : a)
    {
        if (orient_sign(b[0], b[1], p) > 0 && orient_sign(b[1], b[2], p) > 0 &&
            orient_sign(b[2], b[0], p) > 0)
            return false;
    }
    for (const Vec2& p : b)
    {
        if (orient_sign(a[0], a[1], p) > 0 && orient_sign(a[1], a[2], p) > 0 &&
            orient_sign(a[2], a[0], p) > 0)
            return false;
    }
    return true;
}

// Every output triangle is CCW, non-degenerate, uses only input vertices, and
// triangle interiors are pairwise disjoint (MSH-003).
[[nodiscard]] inline bool triangulation_valid(const Polygon& input,
                                              const std::vector<Polygon>& out) noexcept
{
    if (out.empty())
        return false;
    for (const Polygon& t : out)
    {
        if (t.vertices.size() != 3)
            return false;
        const float twice = cross(t.vertices[1] - t.vertices[0],
                                  t.vertices[2] - t.vertices[0]);
        if (twice <= kEps * kEps)
            return false;
        for (const Vec2& v : t.vertices)
        {
            bool found = false;
            for (const Vec2& u : input.vertices)
                found = found || v == u;
            if (!found)
                return false;
        }
    }
    for (std::size_t i = 0; i < out.size(); ++i)
        for (std::size_t j = i + 1; j < out.size(); ++j)
            if (!triangle_interiors_disjoint(out[i], out[j]))
                return false;
    return true;
}

} // namespace vwmini::testing
