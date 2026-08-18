#pragma once

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace vwmini::internal {

constexpr float kEpsilon = 1e-4f;

inline bool finite(Vec2 p) noexcept
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

inline bool finite(float v) noexcept
{
    return std::isfinite(v);
}

// Twice the signed area (cross product) of (b-a, c-a); positive for CCW.
inline float triArea2(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return cross(b - a, c - a);
}

// Strict interior point-in-CCW-triangle test (all barycentric weights positive).
inline bool pointInTri(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return triArea2(a, b, p) > 0.0f && triArea2(b, c, p) > 0.0f && triArea2(c, a, p) > 0.0f;
}

// Proper (open-segment) intersection of segments ab and cd.
inline bool properIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const float o1 = triArea2(a, b, c);
    const float o2 = triArea2(a, b, d);
    const float o3 = triArea2(c, d, a);
    const float o4 = triArea2(c, d, b);
    return ((o1 > 0.0f && o2 < 0.0f) || (o1 < 0.0f && o2 > 0.0f)) &&
           ((o3 > 0.0f && o4 < 0.0f) || (o3 < 0.0f && o4 > 0.0f));
}

// Distance from point to the closed finite segment [a,b].
inline float distPointSegment(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const Vec2 ab = b - a;
    const float len2 = dot(ab, ab);
    if (len2 == 0.0f) {
        return length(p - a);
    }
    const float t = std::clamp(dot(p - a, ab) / len2, 0.0f, 1.0f);
    return length(p - (a + ab * t));
}

// True when the Euclidean distance between two points is at most kEpsilon.
inline bool distLe(Vec2 a, Vec2 b) noexcept
{
    const Vec2 d = a - b;
    return std::sqrt(d.x * d.x + d.y * d.y) <= kEpsilon;
}

// Number of interior samples for a contained-segment check so that spacing is
// <= kEpsilon, bounded so long segments remain cheap (spacing grows past
// kEpsilon only for very long segments; transversal exits are still caught).
inline int segmentSamples(Vec2 a, Vec2 b) noexcept
{
    const float len = length(b - a);
    const int n = static_cast<int>(std::ceil(len / kEpsilon));
    return std::clamp(n, 1, 4096);
}

// True if every interior sample of [a,b] (excluding endpoints) satisfies pred.
// Used for MSH-006 / SIM-002 contained-segment checks: a point is contained
// when strictly inside a triangle OR within kEpsilon of any triangle edge, so
// segments that merely graze a boundary edge within the tolerance pass while
// segments that genuinely exit the mesh are caught.
template <class Pred> inline bool segmentContainedBy(Vec2 a, Vec2 b, Pred pred) noexcept
{
    const int n = segmentSamples(a, b);
    const Vec2 d = b - a;
    for (int i = 1; i <= n; ++i) {
        const Vec2 q = a + d * (static_cast<float>(i) / static_cast<float>(n + 1));
        if (!pred(q)) {
            return false;
        }
    }
    return true;
}

// True if p lies on the closed finite segment [a,b] within kEpsilon.
inline bool pointOnSegment(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    return distPointSegment(p, a, b) <= kEpsilon;
}

// Positive-length collinear overlap of finite segments ab and cd, allowing
// shared endpoints. Returns false when the segments are not collinear within
// kEpsilon or the overlap length is at most kEpsilon.
inline bool collinearOverlap(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const Vec2 dir = b - a;
    const float len = length(dir);
    if (len <= 0.0f) {
        return false;
    }
    // c and d within kEpsilon of the line through a-b (|cross| <= kEpsilon * len).
    if (std::fabs(triArea2(a, b, c)) > kEpsilon * len || std::fabs(triArea2(a, b, d)) > kEpsilon * len) {
        return false;
    }
    const float len2 = dot(dir, dir);
    const float tc = dot(c - a, dir) / len2; // position of c along a-b (a=0, b=1)
    const float td = dot(d - a, dir) / len2;
    const float lo = std::max(0.0f, std::min(tc, td));
    const float hi = std::min(1.0f, std::max(tc, td));
    return (hi - lo) * len > kEpsilon;
}

// Distance between two finite closed segments [a,b] and [c,d].
// Exact for non-parallel interior crossings (convex quadratic minimum) and
// otherwise falls back to the endpoint-to-segment candidates, which cover
// parallel, collinear and degenerate cases.
inline float distSegSeg(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const Vec2 u = b - a;
    const Vec2 v = d - c;
    const Vec2 w = a - c;
    const float A = dot(u, u);
    const float B = dot(u, v);
    const float C = dot(v, v);
    const float D = dot(u, w);
    const float E = dot(v, w);
    const float denom = A * C - B * B;

    float best = std::numeric_limits<float>::infinity();

    if (denom > 1e-9f * A * C) {
        const float t = std::clamp((B * E - C * D) / denom, 0.0f, 1.0f);
        const float uu = std::clamp((A * E - B * D) / denom, 0.0f, 1.0f);
        best = length(w + u * t - v * uu);
    }

    best = std::min(best, distPointSegment(c, a, b));
    best = std::min(best, distPointSegment(d, a, b));
    best = std::min(best, distPointSegment(a, c, d));
    best = std::min(best, distPointSegment(b, c, d));
    return best;
}

} // namespace vwmini::internal
