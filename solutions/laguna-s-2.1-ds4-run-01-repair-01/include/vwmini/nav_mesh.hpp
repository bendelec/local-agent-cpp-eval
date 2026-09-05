#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <vwmini/geometry.hpp>

namespace vwmini {

struct Path;

/** An immutable navigation mesh created from valid CCW triangles. */
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
    // Private representation is implementation-defined. The layout lives in a
    // translation-unit-private detail header consumed only by the mesh sources.
    struct Impl;
    explicit NavMesh(std::shared_ptr<const Impl> impl) noexcept;
    friend Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal);
    std::shared_ptr<const Impl> m_impl{};
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
