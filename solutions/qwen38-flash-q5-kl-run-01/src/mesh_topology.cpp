#include "mesh_topology.hpp"

#include "error.hpp"
#include "predicates.hpp"

#include <algorithm>
#include <expected>
#include <optional>
#include <utility>

namespace vwmini::detail
{
namespace
{

using CellErrors = std::optional<Error>;

/// Reference to one directed edge of a cell: `corner` to `corner + 1`.
struct EdgeRef
{
    std::size_t cell{};
    std::size_t corner{};
};

[[nodiscard]] bool endpoints_coincide(Vec2 a, Vec2 b) noexcept
{
    return length_squared(a - b) <= kEpsilon * kEpsilon;
}

/// True when `point` lies on the cell edge `corner`..`corner+1`, away from both endpoints.
[[nodiscard]] bool point_inside_edge_span(Vec2 point, const Triangle &cell, std::size_t corner)
{
    const Vec2 start = cell.corner(corner);
    const Vec2 end = cell.corner(corner + 1);
    if (endpoints_coincide(point, start) || endpoints_coincide(point, end))
    {
        return false;
    }
    return distance_squared_to_segment(point, start, end) <= kEpsilon * kEpsilon;
}

/// MSH-005: two cells share an edge only when a complete edge matches within epsilon.
[[nodiscard]] bool edges_match(const Triangle &first, std::size_t first_corner,
                               const Triangle &second, std::size_t second_corner)
{
    const Vec2 first_start = first.corner(first_corner);
    const Vec2 first_end = first.corner(first_corner + 1);
    const Vec2 second_start = second.corner(second_corner);
    const Vec2 second_end = second.corner(second_corner + 1);

    const bool aligned =
        endpoints_coincide(first_start, second_start) && endpoints_coincide(first_end, second_end);
    const bool opposed =
        endpoints_coincide(first_start, second_end) && endpoints_coincide(first_end, second_start);
    return aligned || opposed;
}

/// True when `point` is strictly inside `cell` or within epsilon of a closed edge (MSH-006).
[[nodiscard]] bool cell_contains(const Triangle &cell, Vec2 point) noexcept
{
    if (strictly_inside_triangle(point, cell.corner(0), cell.corner(1), cell.corner(2)))
    {
        return true;
    }
    for (std::size_t corner = 0; corner < 3; ++corner)
    {
        if (distance_squared_to_segment(point, cell.corner(corner), cell.corner(corner + 1)) <=
            kEpsilon * kEpsilon)
        {
            return true;
        }
    }
    return false;
}

/// Shape, finiteness, winding and degeneracy of every submitted polygon (MSH-004).
[[nodiscard]] Result<std::vector<Triangle>> collect_cells(const std::vector<Polygon> &submitted)
{
    if (submitted.empty())
    {
        return std::unexpected(
            make_error(ErrorCode::InvalidMesh, "mesh needs at least one triangle"));
    }

    std::vector<Triangle> cells;
    cells.reserve(submitted.size());
    for (const Polygon &polygon : submitted)
    {
        if (polygon.vertices.size() != 3)
        {
            return std::unexpected(make_error(ErrorCode::InvalidMesh,
                                              "only triangles are accepted; use "
                                              "triangulate_simple_polygon first"));
        }
        for (const Vec2 &vertex : polygon.vertices)
        {
            if (!is_finite(vertex))
            {
                return std::unexpected(make_error(ErrorCode::InvalidArgument,
                                                  "mesh triangle has a non-finite coordinate"));
            }
        }
        cells.push_back(Triangle{{polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]}});
    }

    for (const Triangle &cell : cells)
    {
        if (signed_double_area(cell.corner(0), cell.corner(1), cell.corner(2)) <=
            kMinNonDegenerateDoubleArea)
        {
            return std::unexpected(
                make_error(ErrorCode::InvalidMesh, "mesh triangle is clockwise or degenerate"));
        }
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            if (endpoints_coincide(cell.corner(corner), cell.corner(corner + 1)))
            {
                return std::unexpected(
                    make_error(ErrorCode::InvalidMesh, "mesh triangle has a repeated vertex"));
            }
        }
    }
    return cells;
}

/**
 * Derive adjacency and reject non-manifold topology (MSH-005): an edge owned by more than
 * two cells, or two cells sharing more than one edge.
 */
[[nodiscard]] Result<std::vector<std::vector<std::size_t>>>
build_adjacency(const std::vector<Triangle> &cells)
{
    std::vector<EdgeRef> edges;
    edges.reserve(cells.size() * 3);
    for (std::size_t cell = 0; cell < cells.size(); ++cell)
    {
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            edges.push_back(EdgeRef{cell, corner});
        }
    }

    std::vector<std::vector<std::size_t>> neighbours(cells.size());
    std::vector<bool> grouped(edges.size(), false);
    for (std::size_t first = 0; first < edges.size(); ++first)
    {
        if (grouped[first])
        {
            continue;
        }
        grouped[first] = true;
        std::vector<EdgeRef> group{edges[first]};
        for (std::size_t second = first + 1; second < edges.size(); ++second)
        {
            if (!grouped[second] && edges_match(cells[edges[first].cell], edges[first].corner,
                                                cells[edges[second].cell], edges[second].corner))
            {
                grouped[second] = true;
                group.push_back(edges[second]);
            }
        }
        if (group.size() > 2)
        {
            return std::unexpected(make_error(ErrorCode::InvalidMesh,
                                              "a mesh edge belongs to more than two cells "
                                              "(non-manifold)"));
        }
        if (group.size() == 1)
        {
            continue; // Boundary edge, nothing to connect.
        }

        const std::size_t a = group[0].cell;
        const std::size_t b = group[1].cell;
        if (a == b)
        {
            return std::unexpected(
                make_error(ErrorCode::InvalidMesh, "a mesh cell shares an edge with itself"));
        }
        if (std::find(neighbours[a].begin(), neighbours[a].end(), b) != neighbours[a].end())
        {
            return std::unexpected(make_error(ErrorCode::InvalidMesh,
                                              "two mesh cells share more than one edge "
                                              "(non-manifold)"));
        }
        neighbours[a].push_back(b);
        neighbours[b].push_back(a);
    }

    for (std::vector<std::size_t> &list : neighbours)
    {
        std::sort(list.begin(), list.end());
    }
    return neighbours;
}

/// MSH-004: a vertex resting in the middle of another cell's edge is a T-junction.
[[nodiscard]] CellErrors find_t_junction(const std::vector<Triangle> &cells)
{
    for (std::size_t moving = 0; moving < cells.size(); ++moving)
    {
        for (std::size_t other = 0; other != moving; ++other)
        {
            for (std::size_t vertex = 0; vertex < 3; ++vertex)
            {
                const Vec2 point = cells[moving].corner(vertex);
                for (std::size_t edge = 0; edge < 3; ++edge)
                {
                    if (point_inside_edge_span(point, cells[other], edge))
                    {
                        return make_error(ErrorCode::InvalidMesh,
                                          "mesh vertex rests on another cell's edge (T-junction)");
                    }
                }
            }
        }
    }
    return std::nullopt;
}

/// True when the interiors of two triangles intersect (crossing edges or a contained vertex).
[[nodiscard]] bool cells_interfere(const Triangle &first, const Triangle &second)
{
    for (std::size_t a = 0; a < 3; ++a)
    {
        for (std::size_t b = 0; b < 3; ++b)
        {
            if (segments_cross_internally(first.corner(a), first.corner(a + 1), second.corner(b),
                                          second.corner(b + 1)))
            {
                return true;
            }
        }
    }
    for (std::size_t corner = 0; corner < 3; ++corner)
    {
        if (strictly_inside_triangle(first.corner(corner), second.corner(0), second.corner(1),
                                     second.corner(2)) ||
            strictly_inside_triangle(second.corner(corner), first.corner(0), first.corner(1),
                                     first.corner(2)))
        {
            return true;
        }
    }
    return false;
}

/// MSH-004: overlapping triangle interiors are rejected.
[[nodiscard]] CellErrors find_interpenetration(const std::vector<Triangle> &cells)
{
    for (std::size_t first = 0; first + 1 < cells.size(); ++first)
    {
        for (std::size_t second = first + 1; second < cells.size(); ++second)
        {
            if (cells_interfere(cells[first], cells[second]))
            {
                return make_error(ErrorCode::InvalidMesh, "mesh triangle interiors overlap");
            }
        }
    }
    return std::nullopt;
}

} // namespace

Result<MeshTopology> MeshTopology::build(const std::vector<Polygon> &submitted)
{
    auto cells = collect_cells(submitted);
    if (!cells.has_value())
    {
        return std::unexpected(cells.error());
    }
    // Adjacency first so that non-manifold topology is reported before the overlap rules
    // that a non-manifold mesh necessarily also violates.
    auto neighbours = build_adjacency(*cells);
    if (!neighbours.has_value())
    {
        return std::unexpected(neighbours.error());
    }
    if (auto junction = find_t_junction(*cells); junction.has_value())
    {
        return std::unexpected(*junction);
    }
    if (auto interference = find_interpenetration(*cells); interference.has_value())
    {
        return std::unexpected(*interference);
    }
    return MeshTopology(std::move(*cells), std::move(*neighbours));
}

std::optional<std::pair<Vec2, Vec2>> MeshTopology::shared_edge(std::size_t first,
                                                               std::size_t second) const noexcept
{
    for (std::size_t corner = 0; corner < 3; ++corner)
    {
        for (std::size_t other = 0; other < 3; ++other)
        {
            if (edges_match(m_cells[first], corner, m_cells[second], other))
            {
                return std::pair<Vec2, Vec2>{m_cells[first].corner(corner),
                                             m_cells[first].corner(corner + 1)};
            }
        }
    }
    return std::nullopt;
}

bool MeshTopology::contains(Vec2 point) const noexcept
{
    if (!is_finite(point))
    {
        return false;
    }
    for (const Triangle &cell : m_cells)
    {
        if (cell_contains(cell, point))
        {
            return true;
        }
    }
    return false;
}

std::vector<std::size_t> MeshTopology::locate(Vec2 point) const
{
    std::vector<std::size_t> found;
    if (!is_finite(point))
    {
        return found;
    }
    for (std::size_t index = 0; index < m_cells.size(); ++index)
    {
        if (cell_contains(m_cells[index], point))
        {
            found.push_back(index);
        }
    }
    return found;
}

bool MeshTopology::segment_is_contained(Vec2 from, Vec2 to) const
{
    if (!is_finite(from) || !is_finite(to) || !contains(from) || !contains(to))
    {
        return false;
    }

    // The covered part of a segment against a union of convex cells is a union of
    // parameter intervals whose endpoints are edge crossings, so testing the midpoint of
    // every interval between consecutive crossings decides coverage of the whole segment.
    std::vector<float> cuts{0.0f, 1.0f};
    for (const Triangle &cell : m_cells)
    {
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const auto cut = segment_intersection_parameter(from, to, cell.corner(corner),
                                                            cell.corner(corner + 1));
            if (cut.has_value())
            {
                cuts.push_back(*cut);
            }
        }
    }
    std::sort(cuts.begin(), cuts.end());

    const Vec2 delta = to - from;
    float previous = 0.0f;
    for (const float cut : cuts)
    {
        if (cut <= previous)
        {
            continue;
        }
        const Vec2 sample = from + delta * (0.5f * (previous + cut));
        if (!contains(sample))
        {
            return false;
        }
        previous = cut;
    }
    return true;
}

} // namespace vwmini::detail
