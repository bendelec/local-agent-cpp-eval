#pragma once

// The double-precision working vector behind the numeric policy documented in predicates.hpp.
// Converting a float to double is exact, so widening the operands of a computation - never the
// stored coordinates - is what keeps internal geometry meaningful.

#include <vwmini/geometry.hpp>

namespace vwmini::detail
{

/**
 * Double-precision working vector.
 *
 * Implicitly constructed from a `Vec2`: converting a float to double is exact, so widening the
 * operands — never the stored coordinates — is what makes the arithmetic below safe.
 */
struct Vec2d
{
    double x{};
    double y{};

    constexpr Vec2d() noexcept = default;

    constexpr Vec2d(double x_, double y_) noexcept : x(x_), y(y_) {}

    // NOLINTNEXTLINE(google-explicit-constructor) -- widening widens, never loses information.
    constexpr Vec2d(Vec2 value) noexcept
        : x(static_cast<double>(value.x)), y(static_cast<double>(value.y))
    {
    }

    [[nodiscard]] constexpr Vec2 to_vec2() const noexcept
    {
        return Vec2{static_cast<float>(x), static_cast<float>(y)};
    }
};

[[nodiscard]] constexpr Vec2d operator-(Vec2d value) noexcept
{
    return {-value.x, -value.y};
}

[[nodiscard]] constexpr Vec2d operator+(Vec2d a, Vec2d b) noexcept
{
    return {a.x + b.x, a.y + b.y};
}

[[nodiscard]] constexpr Vec2d operator-(Vec2d a, Vec2d b) noexcept
{
    return {a.x - b.x, a.y - b.y};
}

[[nodiscard]] constexpr Vec2d operator*(Vec2d a, double scale) noexcept
{
    return {a.x * scale, a.y * scale};
}

[[nodiscard]] constexpr Vec2d operator/(Vec2d a, double divisor) noexcept
{
    return {a.x / divisor, a.y / divisor};
}

} // namespace vwmini::detail
