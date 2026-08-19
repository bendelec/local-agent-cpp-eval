#include <vwmini/geometry.hpp>

#include "internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace vwmini {

float length(Vec2 value) noexcept
{
    // Scaled norm: factor out the largest component so x*x + y*y cannot
    // overflow to infinity for finite coordinates (e.g. 1e38).
    const float ax = std::fabs(value.x);
    const float ay = std::fabs(value.y);
    const float m = std::max(ax, ay);
    if (m == 0.0f) {
        return 0.0f;
    }
    const float nx = ax / m;
    const float ny = ay / m;
    return m * std::sqrt(nx * nx + ny * ny);
}

Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    if (len == 0.0f) {
        return {0.0f, 0.0f};
    }
    return {value.x / len, value.y / len};
}

namespace {

inline float signed_area(const std::vector<Vec2>& v)
{
    float area = 0.0f;
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        area += a.x * b.y - a.y * b.x;
    }
    return 0.5f * area;
}

} // namespace

Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    const std::vector<Vec2>& v = polygon.vertices;
    const std::size_t n = v.size();

    // Non-finite coordinates -> InvalidArgument.
    for (const Vec2& p : v) {
        if (!internal::finite(p)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite polygon coordinate"});
        }
    }

    if (n < 3) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "fewer than three vertices"});
    }

    // Cyclic consecutive duplicates (including equal first and last).
    for (std::size_t i = 0; i < n; ++i) {
        if (v[i] == v[(i + 1) % n]) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "consecutive duplicate vertices"});
        }
    }

    const float area = signed_area(v);
    // Clockwise winding -> negative area.
    if (area <= 0.0f) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "clockwise or degenerate outline"});
    }

    // Self-intersection check: every pair of non-adjacent edges must not intersect.
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        for (std::size_t j = i + 1; j < n; ++j) {
            // skip adjacent edges
            if (j == i + 1 || (i == 0 && j == n - 1)) {
                continue;
            }
            const Vec2 c = v[j];
            const Vec2 d = v[(j + 1) % n];
            // Any contact between non-adjacent edges is a self-touch: a proper
            // crossing, an endpoint lying on the other edge, or a positive-length
            // collinear overlap. Such outlines are not simple and are rejected.
            if (internal::properIntersect(a, b, c, d) || internal::pointOnSegment(a, c, d) ||
                internal::pointOnSegment(b, c, d) || internal::pointOnSegment(c, a, b) ||
                internal::pointOnSegment(d, a, b) || internal::collinearOverlap(a, b, c, d)) {
                return std::unexpected(Error{ErrorCode::InvalidMesh, "self-intersecting outline"});
            }
        }
    }

    // Ear-clipping with deterministic iteration order.
    std::vector<Vec2> work = v;
    std::vector<bool> removed(n, false);
    std::size_t remaining = n;
    std::vector<Polygon> out;

    auto prev_index = [&](std::size_t i) -> std::size_t {
        std::size_t p = (i + n - 1) % n;
        while (removed[p])
            p = (p + n - 1) % n;
        return p;
    };
    auto next_index = [&](std::size_t i) -> std::size_t {
        std::size_t q = (i + 1) % n;
        while (removed[q])
            q = (q + 1) % n;
        return q;
    };

    while (remaining > 3) {
        bool clipped = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (removed[i])
                continue;
            const std::size_t p = prev_index(i);
            const std::size_t q = next_index(i);
            const Vec2 a = work[p], b = work[i], c = work[q];
            const float ar = internal::triArea2(a, b, c);
            if (ar <= 0.0f)
                continue; // reflex or degenerate ear
            // check no other vertex lies inside the ear triangle
            bool clear = true;
            for (std::size_t k = 0; k < n; ++k) {
                if (k == p || k == i || k == q || removed[k])
                    continue;
                if (internal::pointInTri(work[k], a, b, c)) {
                    clear = false;
                    break;
                }
            }
            if (!clear)
                continue;
            out.push_back(Polygon{{a, b, c}});
            removed[i] = true;
            --remaining;
            clipped = true;
            break;
        }
        if (!clipped) {
            // Should not happen for a valid simple polygon; be safe.
            return std::unexpected(Error{ErrorCode::InvalidMesh, "triangulation failed"});
        }
    }

    // Final triangle of the three remaining vertices.
    std::size_t i0 = 0;
    while (removed[i0])
        ++i0;
    std::size_t i1 = (i0 + 1) % n;
    while (removed[i1])
        i1 = (i1 + 1) % n;
    std::size_t i2 = (i1 + 1) % n;
    while (removed[i2])
        i2 = (i2 + 1) % n;
    out.push_back(Polygon{{work[i0], work[i1], work[i2]}});

    return out;
}

} // namespace vwmini
