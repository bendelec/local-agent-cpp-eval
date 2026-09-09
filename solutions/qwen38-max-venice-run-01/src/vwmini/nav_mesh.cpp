#include <vwmini/nav_mesh.hpp>

#include "vwmini/internal/errors.hpp"
#include "vwmini/internal/mesh_data.hpp"
#include "vwmini/internal/predicates.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <map>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

using internal::Cell;
using internal::Index3;
using internal::cell_contains;
using internal::distance_point_segment;
using internal::is_finite;
using internal::kEpsD;
using internal::kMinArea;
using internal::make_error;
using internal::near;
using internal::next_corner;
using internal::orient;
using internal::point_in_triangle_strict;
using internal::segments_properly_cross;

Vec2 edge_from(const Cell& c, std::size_t k) noexcept
{
    return c[k];
}
Vec2 edge_to(const Cell& c, std::size_t k) noexcept
{
    return c[next_corner(k)];
}

// -- Stage 1: per-input validation, producing clean CCW cells (MSH-004) -------

[[nodiscard]] std::expected<std::vector<Cell>, Error>
prepare_cells(std::span<const Polygon> triangles)
{
    if (triangles.empty()) {
        return make_error(ErrorCode::InvalidMesh, "mesh has no triangles");
    }
    // Non-finite input takes precedence over shape errors, so scan everything first.
    const auto has_non_finite_vertex = [](const Polygon& t) {
        return std::ranges::any_of(t.vertices, [](Vec2 v) { return !is_finite(v); });
    };
    if (std::ranges::any_of(triangles, has_non_finite_vertex)) {
        return make_error(ErrorCode::InvalidArgument, "mesh has a non-finite vertex");
    }

    std::vector<Cell> cells;
    cells.reserve(triangles.size());
    for (const Polygon& t : triangles) {
        if (t.vertices.size() != 3) {
            return make_error(ErrorCode::InvalidMesh, "mesh polygon is not a triangle");
        }
        const Cell c{t.vertices[0], t.vertices[1], t.vertices[2]};
        if (orient(c[0], c[1], c[2]) <= kMinArea) {
            return make_error(ErrorCode::InvalidMesh, "triangle is clockwise or degenerate");
        }
        cells.push_back(c);
    }
    return cells;
}

// -- Stage 2: weld vertices within epsilon so edges can be matched exactly ----
//
// Welding is first-match in input order, so identical input always yields identical
// indices (MSH-005 endpoint matching). A cell whose distinct input corners weld to
// one vertex has an edge shorter than epsilon; it is numerically ambiguous and
// rejected as degenerate.

[[nodiscard]] std::expected<std::vector<Index3>, Error> weld_vertices(std::span<const Cell> cells)
{
    std::vector<Vec2> points;
    std::vector<Index3> index;
    index.reserve(cells.size());
    for (const Cell& c : cells) {
        Index3 row{};
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec2 v = c[k];
            const auto it = std::ranges::find_if(points, [v](Vec2 p) { return near(p, v); });
            if (it == points.end()) {
                points.push_back(v);
                row[k] = static_cast<std::int32_t>(points.size()) - 1;
            } else {
                row[k] = static_cast<std::int32_t>(it - points.begin());
            }
        }
        if (row[0] == row[1] || row[1] == row[2] || row[0] == row[2]) {
            return make_error(ErrorCode::InvalidMesh,
                              "triangle has vertices coincident within epsilon");
        }
        index.push_back(row);
    }
    return index;
}

// -- Stage 3: reject overlapping triangle interiors (MSH-004) ----------------

// Interiors of two CCW cells overlap when a corner of one lies strictly inside the
// other or two edges cross properly. Shared edges and shared corners trigger
// neither; contact along collinear boundaries only (duplicated cells, or an edge
// lying along another cell's edge) is rejected by stage 4 or stage 5 instead.
[[nodiscard]] bool interiors_overlap(const Cell& a, const Cell& b) noexcept
{
    for (std::size_t k = 0; k < 3; ++k) {
        if (point_in_triangle_strict(a[k], b[0], b[1], b[2]) ||
            point_in_triangle_strict(b[k], a[0], a[1], a[2])) {
            return true;
        }
    }
    for (std::size_t e1 = 0; e1 < 3; ++e1) {
        for (std::size_t e2 = 0; e2 < 3; ++e2) {
            if (segments_properly_cross(edge_from(a, e1), edge_to(a, e1), edge_from(b, e2),
                                        edge_to(b, e2))) {
                return true;
            }
        }
    }
    return false;
}

[[nodiscard]] std::expected<void, Error> check_overlap(std::span<const Cell> cells)
{
    for (std::size_t a = 0; a < cells.size(); ++a) {
        for (std::size_t b = a + 1; b < cells.size(); ++b) {
            if (interiors_overlap(cells[a], cells[b])) {
                return make_error(ErrorCode::InvalidMesh, "triangle interiors overlap");
            }
        }
    }
    return {};
}

// -- Stage 4: reject T-junctions, a vertex inside another cell's edge (MSH-004)

[[nodiscard]] std::expected<void, Error> check_t_junctions(std::span<const Cell> cells)
{
    for (std::size_t i = 0; i < cells.size(); ++i) {
        for (std::size_t j = 0; j < cells.size(); ++j) {
            if (i == j) {
                // A cell's own apex within epsilon of its opposite edge makes a
                // thin but non-degenerate triangle, not a T-junction.
                continue;
            }
            for (std::size_t k = 0; k < 3; ++k) {
                const Vec2 v = cells[i][k];
                for (std::size_t e = 0; e < 3; ++e) {
                    const Vec2 p = edge_from(cells[j], e);
                    const Vec2 q = edge_to(cells[j], e);
                    if (distance_point_segment(v, p, q) <= kEpsD && !near(v, p) && !near(v, q)) {
                        return make_error(ErrorCode::InvalidMesh, "mesh has a T-junction");
                    }
                }
            }
        }
    }
    return {};
}

// -- Stage 5: manifold edge matching + adjacency (MSH-004, MSH-005) ---------

[[nodiscard]] std::expected<std::vector<Index3>, Error>
build_adjacency(std::span<const Index3> index)
{
    // One directed use of an undirected welded edge.
    struct EdgeUse {
        std::int32_t cell;
        std::int32_t edge;
        std::int32_t from;
        std::int32_t to;
    };

    // Ordered map so the first reported error is deterministic across runs.
    std::map<std::pair<std::int32_t, std::int32_t>, std::vector<EdgeUse>> groups;
    for (std::size_t i = 0; i < index.size(); ++i) {
        for (std::size_t k = 0; k < 3; ++k) {
            const std::int32_t from = index[i][k];
            const std::int32_t to = index[i][next_corner(k)];
            const auto key = std::pair{std::min(from, to), std::max(from, to)};
            groups[key].push_back(
                EdgeUse{static_cast<std::int32_t>(i), static_cast<std::int32_t>(k), from, to});
        }
    }

    std::vector<Index3> adjacency(index.size(), Index3{-1, -1, -1});
    for (const auto& [key, uses] : groups) {
        if (uses.size() == 1) {
            continue; // boundary edge: adjacency already -1
        }
        if (uses.size() > 2) {
            return make_error(ErrorCode::InvalidMesh, "non-manifold edge shared by 3+ triangles");
        }
        const auto [c1, e1, f1, t1] = uses[0];
        const auto [c2, e2, f2, t2] = uses[1];
        // Two CCW cells sharing a complete edge traverse it in opposite directions;
        // same-direction uses mean duplicated or overlapping cells.
        const bool opposite = (f1 == t2) && (t1 == f2);
        if (!opposite) {
            return make_error(ErrorCode::InvalidMesh,
                              "two triangles share an edge in the same direction");
        }
        adjacency[static_cast<std::size_t>(c1)][static_cast<std::size_t>(e1)] = c2;
        adjacency[static_cast<std::size_t>(c2)][static_cast<std::size_t>(e2)] = c1;
    }
    return adjacency;
}

} // namespace

// -- NavMesh members --------------------------------------------------------

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl))
{
}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    auto cells_result = prepare_cells(triangles);
    if (!cells_result) {
        return std::unexpected(std::move(cells_result).error());
    }
    std::vector<Cell> cells = std::move(*cells_result);

    auto index_result = weld_vertices(cells);
    if (!index_result) {
        return std::unexpected(std::move(index_result).error());
    }
    if (auto overlap = check_overlap(cells); !overlap) {
        return std::unexpected(std::move(overlap).error());
    }
    if (auto junction = check_t_junctions(cells); !junction) {
        return std::unexpected(std::move(junction).error());
    }
    auto adjacency = build_adjacency(*index_result);
    if (!adjacency) {
        return std::unexpected(std::move(adjacency).error());
    }

    // Published only after every stage passes: a failed create exposes no partial
    // mesh and a successful one is never mutated afterwards (MSH-007).
    auto impl = std::make_shared<Impl>();
    impl->cells = std::move(cells);
    impl->adjacency = std::move(*adjacency);
    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept
{
    // A moved-from mesh (null impl) and a non-finite point are never contained.
    if (!m_impl || !is_finite(point)) {
        return false;
    }
    return std::ranges::any_of(m_impl->cells,
                               [point](const Cell& c) { return cell_contains(point, c); });
}

std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl ? m_impl->cells.size() : 0;
}

} // namespace vwmini
