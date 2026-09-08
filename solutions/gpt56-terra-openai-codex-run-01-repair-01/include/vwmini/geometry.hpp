#pragma once

#include <expected>
#include <string>
#include <vector>

namespace vwmini {

/** A two-dimensional point or vector, measured in metres. */
struct Vec2 {
    float x{};
    float y{};

    constexpr bool operator==(const Vec2 &) const noexcept = default;
};

[[nodiscard]] constexpr Vec2 operator+(Vec2 a, Vec2 b) noexcept {
    return {a.x + b.x, a.y + b.y};
}

[[nodiscard]] constexpr Vec2 operator-(Vec2 a, Vec2 b) noexcept {
    return {a.x - b.x, a.y - b.y};
}

[[nodiscard]] constexpr Vec2 operator*(Vec2 value, float scalar) noexcept {
    return {value.x * scalar, value.y * scalar};
}

[[nodiscard]] constexpr Vec2 operator*(float scalar, Vec2 value) noexcept {
    return value * scalar;
}

[[nodiscard]] constexpr float dot(Vec2 a, Vec2 b) noexcept {
    return a.x * b.x + a.y * b.y;
}

[[nodiscard]] constexpr float cross(Vec2 a, Vec2 b) noexcept {
    return a.x * b.y - a.y * b.x;
}

/** Returns Euclidean vector length; finite input yields a finite non-negative result. */
[[nodiscard]] float length(Vec2 value) noexcept;
/** Returns a unit vector, or `{0,0}` when the input has zero length. */
[[nodiscard]] Vec2 normalized(Vec2 value) noexcept;

/** A polygon ring whose vertices are given in order. */
struct Polygon {
    std::vector<Vec2> vertices;
};

/** Stable category for a failed public operation. */
enum class ErrorCode {
    InvalidArgument, ///< A scalar or value violates its documented precondition.
    InvalidMesh,     ///< Finite polygon or triangle-mesh geometry is invalid.
    OutsideMesh,     ///< A finite point is not contained by the navmesh.
    NoPath,          ///< Valid in-mesh endpoints have no connecting route.
    NotFound,        ///< An agent id is unknown or has been removed.
};

/** Diagnosed failure returned through `Result<T>`. */
struct Error {
    ErrorCode code{};
    std::string message;
};

/** Value or diagnosed error returned by fallible public operations. */
template <class T> using Result = std::expected<T, Error>;

/**
 * Validates and triangulates one simple, counter-clockwise, hole-free polygon.
 *
 * Returns `InvalidArgument` for non-finite coordinates and `InvalidMesh` for other
 * invalid outlines. Output is deterministic for identical input on one platform.
 */
[[nodiscard]] Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon &polygon);

} // namespace vwmini
