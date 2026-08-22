#include "vwmini/geometry_priv.hpp"

#include <algorithm>

namespace vwmini::detail {

float polygon_area(const Polygon& p) noexcept
{
    const std::size_t n = p.vertices.size();
    if (n < 3)
        return 0.0f;
    float twice = 0.0f;
    for (std::size_t i = 0; i < n; ++i)
    {
        const Vec2& a = p.vertices[i];
        const Vec2& b = p.vertices[(i + 1) % n];
        twice += a.x * b.y - a.y * b.x;
    }
    return twice * 0.5f;
}

float point_segment_distance(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const Vec2 ab = b - a;
    const float denom = dot(ab, ab);
    if (denom == 0.0f)
        return length(p - a);
    const float t = std::clamp(dot(p - a, ab) / denom, 0.0f, 1.0f);
    return length(p - (a + ab * t));
}

bool point_on_segment_exact(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    if (cross(b - a, p - a) != 0.0f)
        return false;
    const float d2 = squared_distance(a, b);
    if (d2 == 0.0f)
        return p == a;
    const float t = dot(p - a, b - a) / d2;
    return t >= 0.0f && t <= 1.0f;
}

bool segments_properly_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const float o1 = cross(b - a, c - a);
    const float o2 = cross(b - a, d - a);
    const float o3 = cross(d - c, a - c);
    const float o4 = cross(d - c, b - c);
    const bool straddle1 = (o1 > 0.0f && o2 < 0.0f) || (o1 < 0.0f && o2 > 0.0f);
    const bool straddle2 = (o3 > 0.0f && o4 < 0.0f) || (o3 < 0.0f && o4 > 0.0f);
    return straddle1 && straddle2;
}

bool segments_collinear_overlap(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    if (cross(b - a, c - a) != 0.0f || cross(b - a, d - a) != 0.0f)
        return false;
    // Project onto the axis with non-zero span to avoid a degenerate axis.
    const float use_x = std::abs(b.x - a.x) >= std::abs(b.y - a.y);
    const auto span = [use_x](Vec2 p, Vec2 q) {
        const float lo = use_x ? std::min(p.x, q.x) : std::min(p.y, q.y);
        const float hi = use_x ? std::max(p.x, q.x) : std::max(p.y, q.y);
        return std::pair{lo, hi};
    };
    const auto [l1, h1] = span(a, b);
    const auto [l2, h2] = span(c, d);
    return std::min(h1, h2) > std::max(l1, l2);
}

} // namespace vwmini::detail
