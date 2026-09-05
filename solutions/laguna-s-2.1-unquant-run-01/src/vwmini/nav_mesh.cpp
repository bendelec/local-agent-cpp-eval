#include <vwmini/nav_mesh.hpp>

#include <vwmini/detail.hpp>
#include <vwmini/navmesh_detail.hpp>

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace vwmini {

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept
    : m_impl(std::move(impl))
{
}

namespace {

inline std::array<Vec2, 2> tri_edge(const std::array<Vec2, 3>& t, int e) noexcept
{
    return {t[e], t[(e + 1) % 3]};
}

// True when two directed edges share the same unordered endpoints within epsilon.
inline bool edges_match(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const auto close = [](Vec2 p, Vec2 q) {
        const Vec2 d = p - q;
        return dot(d, d) <= detail::epsilon * detail::epsilon;
    };
    return (close(a, c) && close(b, d)) || (close(a, d) && close(b, c));
}

} // namespace

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    if (triangles.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "empty triangle list"});
    }

    std::vector<std::array<Vec2, 3>> tris;
    tris.reserve(triangles.size());
    for (const auto& poly : triangles) {
        if (poly.vertices.size() != 3) {
            return std::unexpected(
                Error{ErrorCode::InvalidMesh, "polygon is not a triangle"});
        }
        const Vec2 a = poly.vertices[0];
        const Vec2 b = poly.vertices[1];
        const Vec2 c = poly.vertices[2];
        if (!detail::finite(a) || !detail::finite(b) || !detail::finite(c)) {
            return std::unexpected(
                Error{ErrorCode::InvalidArgument, "non-finite triangle vertex"});
        }
        if (!detail::valid_triangle(a, b, c)) {
            return std::unexpected(
                Error{ErrorCode::InvalidMesh, "triangle is clockwise or degenerate"});
        }
        tris.push_back({a, b, c});
    }

    const std::size_t T = tris.size();

    // Pairwise geometry validation: T-junctions, edge crossings, overlaps.
    for (std::size_t i = 0; i < T; ++i) {
        for (std::size_t j = i + 1; j < T; ++j) {
            bool shared_edge_found = false;
            Vec2 se_a{}, se_b{}; // shared edge endpoints (in i's orientation)

            // Locate shared edge and test proper edge crossings.
            for (int e1 = 0; e1 < 3; ++e1) {
                const Vec2 a1 = tris[i][e1];
                const Vec2 b1 = tris[i][(e1 + 1) % 3];
                for (int e2 = 0; e2 < 3; ++e2) {
                    const Vec2 a2 = tris[j][e2];
                    const Vec2 b2 = tris[j][(e2 + 1) % 3];
                    if (edges_match(a1, b1, a2, b2)) {
                        shared_edge_found = true;
                        se_a = a1;
                        se_b = b1;
                        continue;
                    }
                    // Proper crossing of non-adjacent edges => overlap.
                    const bool shares_endpoint =
                        (a1 == a2 || a1 == b2 || b1 == a2 || b1 == b2);
                    if (!shares_endpoint &&
                        detail::proper_segment_intersection(a1, b1, a2, b2)) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                    }
                }
            }

            // T-junction: a vertex in the interior of an edge of the other.
            for (int k = 0; k < 3; ++k) {
                for (int e = 0; e < 3; ++e) {
                    const Vec2 a = tris[j][e];
                    const Vec2 b = tris[j][(e + 1) % 3];
                    if (detail::vertex_in_segment_interior(tris[i][k], a, b)) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "T-junction detected"});
                    }
                    if (detail::vertex_in_segment_interior(tris[j][k], tris[i][e],
                                                           tris[i][(e + 1) % 3])) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "T-junction detected"});
                    }
                }
            }

            if (shared_edge_found) {
                // Same-side apexes => two CCW triangles folding over a shared edge
                // => overlapping interiors.
                const Vec2 apex_i = [&] {
                    for (int k = 0; k < 3; ++k) {
                        if (dot(tris[i][k] - se_a, tris[i][k] - se_a) > detail::epsilon_sq &&
                            dot(tris[i][k] - se_b, tris[i][k] - se_b) > detail::epsilon_sq)
                            return tris[i][k];
                    }
                    return tris[i][0];
                }();
                const Vec2 apex_j = [&] {
                    for (int k = 0; k < 3; ++k) {
                        if (dot(tris[j][k] - se_a, tris[j][k] - se_a) > detail::epsilon_sq &&
                            dot(tris[j][k] - se_b, tris[j][k] - se_b) > detail::epsilon_sq)
                            return tris[j][k];
                    }
                    return tris[j][0];
                }();
                const float s_i = cross(se_b - se_a, apex_i - se_a);
                const float s_j = cross(se_b - se_a, apex_j - se_a);
                if (s_i == 0.0f || s_j == 0.0f || s_i * s_j > 0.0f) {
                    return std::unexpected(
                        Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                }
            } else {
                // No shared edge: overlap iff a vertex of one lies strictly inside
                // the other triangle.
                for (int k = 0; k < 3; ++k) {
                    if (detail::point_in_triangle_strict(tris[j][k], tris[i][0],
                                                         tris[i][1], tris[i][2])) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                    }
                }
                for (int k = 0; k < 3; ++k) {
                    if (detail::point_in_triangle_strict(tris[i][k], tris[j][0],
                                                         tris[j][1], tris[j][2])) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                    }
                }
            }
        }
    }

    // Manifold vertex check (MSH-004 T-junction / non-manifold vertex): every
    // shared vertex must have a single connected fan of incident triangles,
    // i.e. its link graph is connected with max degree 2 (a path for boundary
    // vertices, a cycle for interior vertices). Two triangles touching only at
    // a vertex with no bridging triangle fail this and are rejected; disjoint
    // island meshes (no shared vertices, MSH-005) remain valid.
    {
        const std::size_t C = T * 3;
        std::vector<int> corner_vid(C, -1);
        std::vector<Vec2> verts;
        for (std::size_t c = 0; c < C; ++c) {
            const Vec2 p = tris[c / 3][c % 3];
            int id = -1;
            for (std::size_t k = 0; k < verts.size(); ++k) {
                const Vec2 diff = p - verts[k];
                if (dot(diff, diff) <= detail::epsilon_sq) {
                    id = static_cast<int>(k);
                    break;
                }
            }
            if (id < 0) {
                id = static_cast<int>(verts.size());
                verts.push_back(p);
            }
            corner_vid[c] = id;
        }

        const auto idx_of = [](const std::vector<int>& ns, int x) {
            for (std::size_t k = 0; k < ns.size(); ++k) {
                if (ns[k] == x) {
                    return static_cast<int>(k);
                }
            }
            return -1;
        };

        for (std::size_t u = 0; u < verts.size(); ++u) {
            std::vector<int> neighbors;
            std::vector<std::pair<int, int>> ledges;
            for (std::size_t t = 0; t < T; ++t) {
                for (int lv = 0; lv < 3; ++lv) {
                    if (corner_vid[t * 3 + lv] != static_cast<int>(u)) {
                        continue;
                    }
                    const int oa = corner_vid[t * 3 + ((lv + 1) % 3)];
                    const int ob = corner_vid[t * 3 + ((lv + 2) % 3)];
                    if (oa == ob) {
                        continue; // degenerate triangle already rejected
                    }
                    const auto add = [&neighbors](int x) {
                        for (std::size_t k = 0; k < neighbors.size(); ++k) {
                            if (neighbors[k] == x) {
                                return;
                            }
                        }
                        neighbors.push_back(x);
                    };
                    add(oa);
                    add(ob);
                    ledges.emplace_back(oa, ob);
                }
            }
            if (neighbors.size() < 2) {
                continue; // single incident triangle: a valid boundary fan
            }
            std::vector<int> deg(neighbors.size(), 0);
            std::vector<std::vector<int>> adj(neighbors.size());
            for (const auto& e : ledges) {
                const int a = idx_of(neighbors, e.first);
                const int b = idx_of(neighbors, e.second);
                if (a < 0 || b < 0 || a == b) {
                    continue;
                }
                adj[a].push_back(b);
                adj[b].push_back(a);
                ++deg[a];
                ++deg[b];
            }
            bool non_manifold = false;
            for (int d : deg) {
                if (d > 2) {
                    non_manifold = true;
                }
            }
            std::vector<bool> seen(neighbors.size(), false);
            std::vector<int> stack{0};
            seen[0] = true;
            while (!stack.empty()) {
                const int x = stack.back();
                stack.pop_back();
                for (int y : adj[x]) {
                    if (!seen[y]) {
                        seen[y] = true;
                        stack.push_back(y);
                    }
                }
            }
            for (std::size_t k = 0; k < neighbors.size(); ++k) {
                if (!seen[k]) {
                    non_manifold = true;
                }
            }
            if (non_manifold) {
                return std::unexpected(
                    Error{ErrorCode::InvalidMesh, "T-junction or non-manifold vertex"});
            }
        }
    }

    // Build adjacency (each edge has 0 or 1 partner; >1 => non-manifold).
    std::vector<std::array<std::size_t, 3>> neighbours(T);
    for (std::size_t i = 0; i < T; ++i) {
        for (int e = 0; e < 3; ++e) {
            neighbours[i][e] = NavMesh::Impl::no_cell;
        }
    }
    for (std::size_t i = 0; i < T; ++i) {
        for (int e = 0; e < 3; ++e) {
            if (neighbours[i][e] != NavMesh::Impl::no_cell) continue;
            const Vec2 a = tris[i][e];
            const Vec2 b = tris[i][(e + 1) % 3];
            std::size_t count = 0;
            std::size_t partner = NavMesh::Impl::no_cell;
            std::size_t partner_edge = 0;
            for (std::size_t j = 0; j < T; ++j) {
                if (j == i) continue;
                for (int f = 0; f < 3; ++f) {
                    if (edges_match(a, b, tris[j][f], tris[j][(f + 1) % 3])) {
                        ++count;
                        partner = j;
                        partner_edge = f;
                    }
                }
            }
            if (count > 1) {
                return std::unexpected(
                    Error{ErrorCode::InvalidMesh, "non-manifold edge"});
            }
            if (count == 1) {
                neighbours[i][e] = partner;
                neighbours[partner][partner_edge] = i;
            }
        }
    }

    auto impl = std::make_shared<NavMesh::Impl>(std::move(tris), std::move(neighbours));
    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept
{
    if (!detail::finite(point)) {
        return false;
    }
    const auto& tris = m_impl->triangles;
    for (const auto& t : tris) {
        if (detail::contains_point_triangle(point, t[0], t[1], t[2])) {
            return true;
        }
    }
    return false;
}

std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl->triangles.size();
}

} // namespace vwmini
