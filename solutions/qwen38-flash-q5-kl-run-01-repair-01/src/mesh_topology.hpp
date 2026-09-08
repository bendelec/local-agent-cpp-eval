#pragma once

#include <vwmini/geometry.hpp>

#include "vec2d.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>

namespace vwmini::detail
{

/// One accepted walkable triangle, stored as submitted (MSH-008 permits internal
/// transforms but keeping the input verbatim keeps `cell_count()` trivially exact and
/// keeps the validation rules readable).
struct Triangle
{
    std::array<Vec2, 3> vertices{};

    [[nodiscard]] Vec2 corner(std::size_t index) const noexcept
    {
        return vertices[index % 3];
    }

    /// Area centroid in the numeric policy's double domain: adding three float coordinates in
    /// float overflows for extreme coordinates, so the mean is taken in double.
    [[nodiscard]] Vec2d centroid() const noexcept
    {
        const double x = double(vertices[0].x) + double(vertices[1].x) + double(vertices[2].x);
        const double y = double(vertices[0].y) + double(vertices[1].y) + double(vertices[2].y);
        return {x / 3.0, y / 3.0};
    }
};

/**
 * Immutable, already-validated triangle set with derived adjacency (MSH-004/005) and the
 * single implementation of the walkability question (MSH-006).
 *
 * Invariant: every cell is a finite, non-degenerate, counter-clockwise triangle with no
 * interior overlap, no T-junction and at most two cells per edge. Built only through
 * `build`, which never leaves a partially constructed mesh behind (MSH-007).
 */
class MeshTopology
{
public:
    MeshTopology() = default;

    /// Validates `triangles` and returns the derived topology, or the contract's error.
    [[nodiscard]] static Result<MeshTopology> build(const std::vector<Polygon> &submitted);

    [[nodiscard]] std::size_t cell_count() const noexcept
    {
        return m_cells.size();
    }

    /// \pre cell_index < cell_count()
    [[nodiscard]] const Triangle &cell(std::size_t cell_index) const noexcept
    {
        return m_cells[cell_index];
    }

    /// Cells sharing a complete edge with `cell_index`, in ascending cell order.
    /// \pre cell_index < cell_count()
    [[nodiscard]] std::span<const std::size_t> neighbours(std::size_t cell_index) const noexcept
    {
        return m_neighbours[cell_index];
    }

    /// MSH-006: strictly inside a cell, or within `kEpsilon` of a cell edge segment.
    [[nodiscard]] bool contains(Vec2 point) const noexcept;

    /// Every cell that `point` is contained by, ascending cell order. Empty when outside.
    [[nodiscard]] std::vector<std::size_t> locate(Vec2 point) const;

    /// The complete edge two adjacent cells share, or `nullopt` when they are not adjacent.
    [[nodiscard]] std::optional<std::pair<Vec2, Vec2>>
    shared_edge(std::size_t first, std::size_t second) const noexcept;

    /// True when every point of the closed segment `from`..`to` satisfies `contains`.
    [[nodiscard]] bool segment_is_contained(Vec2 from, Vec2 to) const;

private:
    MeshTopology(std::vector<Triangle> cells, std::vector<std::vector<std::size_t>> neighbours)
        : m_cells(std::move(cells)), m_neighbours(std::move(neighbours))
    {
    }

    std::vector<Triangle> m_cells;
    std::vector<std::vector<std::size_t>> m_neighbours;
};

} // namespace vwmini::detail
