#pragma once

#include <vwmini/geometry.hpp>

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace vwmini {

struct Path;

/** A single CCW triangle stored by value inside an accepted mesh. */
struct Triangle {
    Vec2 v[3];
};

/** An immutable navigation mesh created from valid triangles. */
class NavMesh {
public:
    NavMesh() = delete;

    /** Validates and creates an immutable mesh from non-empty CCW triangles. */
    [[nodiscard]] static Result<NavMesh> create(std::vector<Polygon> triangles);
    /** True when `point` is in a triangle or within the documented boundary tolerance. */
    [[nodiscard]] bool contains(Vec2 point) const noexcept;
    /** Number of accepted input triangles; constant-time and allocation-free. */
    [[nodiscard]] std::size_t cell_count() const noexcept;

private:
    struct Impl;

    explicit NavMesh(std::shared_ptr<const Impl> impl) noexcept;

    // Controlled seam for pathfinding: exposes accepted triangles without leaking
    // internal representation details. Defined out-of-line where `Impl` is complete.
    [[nodiscard]] std::span<const Triangle> triangles_view() const noexcept;

    std::shared_ptr<const Impl> m_impl;

    friend Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal);
};

/** A contained route polyline, preserving supplied start and goal endpoints. */
struct Path {
    std::vector<Vec2> points;
};

/**
 * Finds a deterministic contained path between two mesh points.
 *
 * Returns `InvalidArgument` for non-finite endpoints, `OutsideMesh` for finite
 * out-of-mesh endpoints, and `NoPath` for disconnected mesh components.
 */
[[nodiscard]] Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal);

} // namespace vwmini
