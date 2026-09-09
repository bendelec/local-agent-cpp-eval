#pragma once

// Double-precision 2D vector used for all internal geometric computation.
// Public values are float (`Vec2`); converting to double once and computing in
// double keeps results robust and bitwise deterministic on one platform
// (SIM-002/004). Header-only and allocation-free.

#include <vwmini/geometry.hpp>

#include <cmath>

namespace vwmini::internal {

/// A 2D vector or point in double precision.
struct D2 {
    double x{};
    double y{};

    constexpr bool operator==(const D2&) const noexcept = default;
};

/// Widens a float vector to double.
[[nodiscard]] inline D2 to_d(Vec2 v) noexcept
{
    return {double(v.x), double(v.y)};
}

/// Rounds a double vector back to float.
[[nodiscard]] inline Vec2 to_vec(D2 v) noexcept
{
    return Vec2{float(v.x), float(v.y)};
}

[[nodiscard]] inline D2 operator+(D2 a, D2 b) noexcept
{
    return {a.x + b.x, a.y + b.y};
}
[[nodiscard]] inline D2 operator-(D2 a, D2 b) noexcept
{
    return {a.x - b.x, a.y - b.y};
}
[[nodiscard]] inline D2 operator*(D2 a, double s) noexcept
{
    return {a.x * s, a.y * s};
}
[[nodiscard]] inline D2 operator*(double s, D2 a) noexcept
{
    return a * s;
}
[[nodiscard]] inline double dot2(D2 a, D2 b) noexcept
{
    return a.x * b.x + a.y * b.y;
}
[[nodiscard]] inline double cross2(D2 a, D2 b) noexcept
{
    return a.x * b.y - a.y * b.x;
}
/// Twice the signed area of (a, b, c); positive when c is left of the ray a->b.
[[nodiscard]] inline double triarea2(D2 a, D2 b, D2 c) noexcept
{
    return cross2(b - a, c - a);
}
/// Euclidean distance between two points.
[[nodiscard]] inline double dist(D2 a, D2 b) noexcept
{
    return std::sqrt(dot2(a - b, a - b));
}
/// Euclidean length of a vector.
[[nodiscard]] inline double length2(D2 a) noexcept
{
    return std::sqrt(dot2(a, a));
}
/// Unit vector, or {0,0} when the input has (near-)zero length.
[[nodiscard]] inline D2 normalized2(D2 a) noexcept
{
    const double len = length2(a);
    return len > 0.0 ? a * (1.0 / len) : D2{};
}

} // namespace vwmini::internal
